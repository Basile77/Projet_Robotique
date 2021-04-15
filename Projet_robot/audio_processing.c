#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/

void analyse_data(float *fourier, uint16_t num_samples){

	uint16_t sum = 0;
	float freq = 0;
	for (uint16_t i = 0; i<num_samples; ++i){
		sum += fourier[i];
	}

	float average = (float)sum/num_samples;

	for (uint16_t i = 512; i<num_samples; ++i){
		if (fourier[i] > 3*average){
			freq = 8000 - (i-512)*8000/512;

			if (freq > 1500){
				left_motor_set_speed(1800);
				right_motor_set_speed(1800);
			}
			if (freq < 1300){
				left_motor_set_speed(-1800);
				right_motor_set_speed(-1800);
			}
		}
	}



}

void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/
	static uint16_t nb_sample = 0;
	static uint8_t mustSend = 0;

	for (uint16_t i = 0; i<num_samples;i+=4){

		micLeft_cmplx_input[nb_sample] = (float)data[i];
		micRight_cmplx_input[nb_sample] = (float)data[i+MIC_LEFT];
		micFront_cmplx_input[nb_sample] = (float)data[i+2];
		micBack_cmplx_input[nb_sample] = (float)data[i+3];

		nb_sample++;

		micLeft_cmplx_input[nb_sample] = 0;
		micRight_cmplx_input[nb_sample] = 0;
		micFront_cmplx_input[nb_sample] = 0;
		micBack_cmplx_input[nb_sample] = 0;

		nb_sample++;

		if (nb_sample > 2*FFT_SIZE){
			break;
		}
	}

	if(nb_sample >= (2 * FFT_SIZE)){
		/* FFT proccessing
		*
		* This FFT function stores the results in the input buffer given.
		* This is an "In Place" function.
		*/
		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);
		/* Magnitude processing
		*
		* Computes the magnitude of the complex numbers and
		* stores them in a buffer of FFT_SIZE because it only contains
		* real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);






		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			chprintf((BaseSequentialStream *) &SDU1, "Bonsoir");
			mustSend = 0;
		}
		nb_sample = 0;
		mustSend++;
		analyse_data(micLeft_output, FFT_SIZE);
	}
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
