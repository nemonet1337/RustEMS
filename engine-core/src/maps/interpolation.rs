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
///
/// Uses a binary search over the monotonically increasing axis — O(log N)
/// instead of a linear scan, which matters in the per-tooth hot path where a
/// single ignition + injection computation performs many axis lookups.
#[inline]
fn find_bin<const N: usize>(bins: &[f32; N], value: f32) -> (usize, f32) {
    // Clamp to first bin
    if value <= bins[0] {
        return (0, 0.0);
    }
    // Clamp to last bin
    if value >= bins[N - 1] {
        return (N - 2, 1.0);
    }

    // Binary search for the greatest i with bins[i] <= value.
    let mut lo = 0usize;
    let mut hi = N - 1;
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if value < bins[mid] {
            hi = mid;
        } else {
            lo = mid;
        }
    }

    let span = bins[lo + 1] - bins[lo];
    let frac = if fabsf(span) < 1e-12 {
        0.0
    } else {
        (value - bins[lo]) / span
    };
    (lo, frac)
}

/// 1-D linear interpolation.
///
/// `bins` and `values` must have the same length `N`.
#[inline]
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
#[inline]
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
    fn binary_search_matches_linear_reference() {
        // Reference linear-scan implementation
        fn find_bin_linear<const N: usize>(bins: &[f32; N], value: f32) -> (usize, f32) {
            if value <= bins[0] {
                return (0, 0.0);
            }
            if value >= bins[N - 1] {
                return (N - 2, 1.0);
            }
            for i in 0..N - 1 {
                if value < bins[i + 1] {
                    let span = bins[i + 1] - bins[i];
                    let frac = if span.abs() < 1e-12 { 0.0 } else { (value - bins[i]) / span };
                    return (i, frac);
                }
            }
            (N - 2, 1.0)
        }

        let bins = [
            500.0_f32, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 3500.0, 4000.0,
            4500.0, 5000.0, 5500.0, 6000.0, 6500.0, 7000.0, 7500.0, 8000.0,
        ];
        // Sweep across the whole axis including edges and exact bin values
        let mut x = 0.0_f32;
        while x < 9000.0 {
            let (li, lf) = find_bin_linear(&bins, x);
            let (bi, bf) = super::find_bin(&bins, x);
            assert_eq!(li, bi, "index mismatch at x={x}");
            assert!((lf - bf).abs() < 1e-6, "fraction mismatch at x={x}");
            x += 13.7;
        }
        for &edge in &bins {
            let (li, lf) = find_bin_linear(&bins, edge);
            let (bi, bf) = super::find_bin(&bins, edge);
            assert_eq!(li, bi);
            assert!((lf - bf).abs() < 1e-6);
        }
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
