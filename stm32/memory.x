/* STM32F407ZGT6 memory layout (microRusEFI board)
 * Flash: 1 MiB @ 0x0800_0000
 * RAM:   192 KiB @ 0x2000_0000 (SRAM1 128 KiB + SRAM2 64 KiB, contiguous)
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1M
  RAM   : ORIGIN = 0x20000000, LENGTH = 192K
}
