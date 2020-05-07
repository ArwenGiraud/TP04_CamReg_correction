#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

float max_magnitude_define(float* data, uint8_t freqMin, uint8_t freqMax);
void sound_remote(void);
void processAudioData(int16_t *data, uint16_t num_samples);
bool sound_manuel_remote(void);
uint8_t get_state(void);

#endif /* AUDIO_PROCESSING_H */
