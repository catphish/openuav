#include <stdint.h>
#include "gyro.h"

// This struct represents a FIR filter
struct filter {
  // The length of the filter
  int length;
  // The index of the next value in the ring buffer
  int last_index;
  // Pointer to the filter coefficients
  const float (*coeffs)[];
  // Pointer to the ring buffer
  struct gyro_data *history;
};

// FIR filter for Gyro P term
struct filter gyro_filter_p = {
  .length = 17,
  .last_index = 0,
  .coeffs = &(const float[]){
    0.000000000000000000f,
    0.000460990755913804f,
    0.003904853418603426f,
    0.014975629354308056f,
    0.038805516331589775f,
    0.076533957530631885f,
    0.121097318822696309f,
    0.158012278728653421f,
    0.172418910115206325f,
    0.158012278728653421f,
    0.121097318822696351f,
    0.076533957530631941f,
    0.038805516331589796f,
    0.014975629354308051f,
    0.003904853418603431f,
    0.000460990755913805f,
    0.000000000000000000f
  },
  .history = (struct gyro_data[17]){}
};

// FIR filter for Gyro D term
struct filter gyro_filter_d = {
  .length = 47,
  .last_index = 0,
  .coeffs = &(const float[]){
    -0.000000000000000001f,
    0.000064393676763220f,
    0.000271319801257540f,
    0.000649073204382278f,
    0.001235623050647806f,
    0.002076637553675906f,
    0.003222058252059904f,
    0.004721452177155942f,
    0.006618513253826244f,
    0.008945196904421814f,
    0.011716041834802050f,
    0.014923251981835048f,
    0.018533075973444006f,
    0.022483932798082779f,
    0.026686597480116862f,
    0.031026590809771163f,
    0.035368727580917296f,
    0.039563585562856544f,
    0.043455480385182557f,
    0.046891386356592825f,
    0.049730143962839937f,
    0.051851251307310706f,
    0.053162553912562159f,
    0.053606224358990776f,
    0.053162553912562159f,
    0.051851251307310706f,
    0.049730143962839930f,
    0.046891386356592839f,
    0.043455480385182564f,
    0.039563585562856544f,
    0.035368727580917296f,
    0.031026590809771187f,
    0.026686597480116866f,
    0.022483932798082775f,
    0.018533075973443999f,
    0.014923251981835051f,
    0.011716041834802057f,
    0.008945196904421819f,
    0.006618513253826253f,
    0.004721452177155945f,
    0.003222058252059902f,
    0.002076637553675908f,
    0.001235623050647806f,
    0.000649073204382280f,
    0.000271319801257541f,
    0.000064393676763220f,
    -0.000000000000000001f
  },
  .history = (struct gyro_data[47]){}
};

// Apply a FIR filter to a gyro input value
void filter_gyro_data(struct gyro_data *input, struct gyro_data *output, struct filter *filter) {
  // Initialize output
  output->x = 0;
  output->y = 0;
  output->z = 0;
  // Add the new value to the ring buffer
  filter->history[filter->last_index] = *input;
  // Multiply each value in the ring buffer by the corresponding coefficient
  for (int i = 0; i < filter->length; i++) {
    output->x += (*filter->coeffs)[i] * filter->history[(filter->last_index + i) % filter->length].x;
    output->y += (*filter->coeffs)[i] * filter->history[(filter->last_index + i) % filter->length].y;
    output->z += (*filter->coeffs)[i] * filter->history[(filter->last_index + i) % filter->length].z;
  }
  // Increment the ring buffer index
  filter->last_index = (filter->last_index + 1) % filter->length;
}

// Filter a gyro input value using the P filter
void filter_gyro_p(struct gyro_data *input, struct gyro_data *output) {
  filter_gyro_data(input, output, &gyro_filter_p);
}

// Filter a gyro input value using the D filter
void filter_gyro_d(struct gyro_data *input, struct gyro_data *output) {
  filter_gyro_data(input, output, &gyro_filter_d);
}
