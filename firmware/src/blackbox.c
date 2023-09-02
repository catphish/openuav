#include "flash.h"
#include "blackbox.h"

// These variables track the current page and offset within the page. These
// variables are continuously incremented as frames are written to the flash.
static volatile uint16_t page_offset;
static volatile uint32_t page;

// Initialize the ADC by finding the first free page in the flash.
void blackbox_init() {
  page_offset = 0;
  page = blackbox_find_free_page();
}

// Bisect the flash to find the first free page.
uint16_t blackbox_find_free_page() {
  uint32_t low = 0;
  uint32_t high = 65535;
  uint32_t mid = 0;
  while(low < high) {
    mid = (low + high) / 2;
    flash_page_read(mid);
    uint32_t data;
    flash_read((uint8_t*)&data, 4, 0);
    if(data == 0xFFFFFFFF) {
      high = mid;
    } else {
      low = mid + 1;
    }
  }
  return low;
}

int skip_next_frame = 0;
// Write a 64 byte frame to the flash chip. If the page is full, commit it to
// the flash and increment the page number.
void blackbox_write(struct blackbox_frame * frame) {
  if(page == 0xffff) return;

  // If the skip next frame flag is set, skip this frame and clear the flag.
  // This allows us to skip the next write after each commit as a workaround
  // for not being able to write to the flash while it is busy committing the
  // previous page.
  if(skip_next_frame) {
    skip_next_frame = 0;
    return;
  }

  // Write the frame to the flash chip.
  flash_program_load((uint8_t*)frame, page_offset, sizeof(struct blackbox_frame));
  page_offset += 64;

  // If the page is full, commit it to the flash and increment page number.
  if(page_offset == 2048) {
    flash_program_execute(page);
    page++;
    page_offset = 0;
    // Set the skip next frame flag to skip the next frame write.
    skip_next_frame = 1;
  }
}