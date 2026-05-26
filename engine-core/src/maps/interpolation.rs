//! 1D and 2D linear interpolation for lookup tables.
//!
//! All tables use fixed-size arrays to avoid heap allocation.

use libm::fabsf;

/// Linearly interpolate between two f32 values.
///
/// Returns `y0` when `t == 0.0`, `y1` when `t == 1.0`.
#[inline]
pub fn lerp(y0: f32, y1: f32, t: f32) -> f32 {
    y0 + (y1 - y0) * t
}

/// Find the bin index and fractional position for `value` within `bins`.
///
/// Returns `(index, fraction)` where `index` is the lower bin index and
/// `fraction` is in `[0.0, 1.0]`.  Clamps to the ends of the array.
fn find_bin<const N: usize>(bins: &[f32; N], value: f32) -> (usize, f32) {
    // Clamp to first bin
    if value <= bins[0] {
        return (0, 0.0);
    }
    // Clamp to last bin
    if value >= bins[N - 1] {
        return (N - 2, 1.0);
    }

    for i in 0..N - 1 {
        if value < bins[i + 1] {
            let span = bins[i + 1] - bins[i];
            let frac = if fabsf(span) < 1e-12 {
                0.0
            } else {
                (value - bins[i]) / span
            };
            return (i, frac);
        }
    }

    // Should never reach here due to clamping above
    (N - 2, 1.0)
}

/// 1-D linear interpolation.
///
/// `bins` and `values` must have the same length `N`.
pub fn interpolate1d<const N: usize>(bins: &[f32; N], values: &[f32; N], x: f32) -> f32 {
    let (i, t) = find_bin(bins, x);
    lerp(values[i], values[i + 1], t)
}

/// 2-D bilinear interpolation.
///
/// `table[row][col]` where rows correspond to `row_bins` (e.g. load axis)
/// and columns correspond to `col_bins` (e.g. RPM axis).
///
/// `R` = number of rows, `C` = number of columns.
pub fn interpolate2d<const R: usize, const C: usize>(
    table: &[[f32; C]; R],
    row_bins: &[f32; R],
    row_val: f32,
    col_bins: &[f32; C],
    col_val: f32,
) -> f32 {
    let (ri, rt) = find_bin(row_bins, row_val);
    let (ci, ct) = find_bin(col_bins, col_val);

    // Bilinear interpolation
    let v00 = table[ri][ci];
    let v01 = table[ri][ci + 1];
    let v10 = table[ri + 1][ci];
    let v11 = table[ri + 1][ci + 1];

    let top = lerp(v00, v01, ct);
    let bot = lerp(v10, v11, ct);
    lerp(top, bot, rt)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn lerp_endpoints() {
        assert_relative_eq!(lerp(0.0, 10.0, 0.0), 0.0);
        assert_relative_eq!(lerp(0.0, 10.0, 1.0), 10.0);
        assert_relative_eq!(lerp(0.0, 10.0, 0.5), 5.0);
    }

    #[test]
    fn interpolate1d_clamps() {
        let bins = [0.0_f32, 10.0, 20.0];
        let vals = [0.0_f32, 5.0, 10.0];
        assert_relative_eq!(interpolate1d(&bins, &vals, -5.0), 0.0);
        assert_relative_eq!(interpolate1d(&bins, &vals, 25.0), 10.0);
        assert_relative_eq!(interpolate1d(&bins, &vals, 10.0), 5.0);
        assert_relative_eq!(interpolate1d(&bins, &vals, 15.0), 7.5);
    }

    #[test]
    fn interpolate2d_corners() {
        let table = [[0.0_f32, 10.0], [20.0, 30.0]];
        let row_bins = [0.0_f32, 1.0];
        let col_bins = [0.0_f32, 1.0];
        assert_relative_eq!(interpolate2d(&table, &row_bins, 0.0, &col_bins, 0.0), 0.0);
        assert_relative_eq!(interpolate2d(&table, &row_bins, 0.0, &col_bins, 1.0), 10.0);
        assert_relative_eq!(interpolate2d(&table, &row_bins, 1.0, &col_bins, 0.0), 20.0);
        assert_relative_eq!(interpolate2d(&table, &row_bins, 1.0, &col_bins, 1.0), 30.0);
        // Centre
        assert_relative_eq!(interpolate2d(&table, &row_bins, 0.5, &col_bins, 0.5), 15.0);
    }
}
