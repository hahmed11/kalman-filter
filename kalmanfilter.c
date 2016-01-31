#include <stdio.h>
//#include "arm_math.h"

typedef struct kalman_struct{
    float q; // process noise covariance
    float r; // measurement noise covariance
    float x; // estimated value
    float p; // estimation error covariance
    float k; // adaptive kalman filter gain
}kalman_state;

int Kalmanfilter_C(float* InputArray, float* OutputArray, kalman_state* kstate, int Length);
void reset(kalman_state* kinit);
void subtract(float* sub, float* in1, float* in2, int length);
void misc(float* result, float* diff, int length);
float mean(float* input, int length2);
float root(float input);
float squarer(float input);
float correlation(float* in, float* out, int length);
void convolve(float* Result, float* in1, float* in2, int length);

int main(){

    float input[10] = {0.39, 0.50, 0.48, 0.29, 0.25, 0.32, 0.34, 0.48, 0.41, 0.45};
    int len = 10;
    float output[len];

    kalman_state kstate;
    reset(&kstate);
    Kalmanfilter_C(input, output, &kstate, len);
    printf("\n");

    // subtract
	printf("subtraction:\n");
    float temp[len];
    subtract(temp, input, output, len);
    int j;
    for(j = 0; j < len; j++){
        printf("%f\n", temp[j]);
    }

    // misc
	printf("\n");
    float miscresult[2] = {0, 0};
    misc(miscresult, temp, len);
    printf("mean: %f stdev: %f\n", miscresult[0], miscresult[1]);
	
	// correlation
	printf("correlation: %f\n", correlation(input, output, len));
	
	// convolution
	printf("\n");
	float holder[len];
	int i;
	for(i = 0; i < len; i++){
		holder[i] = 0;
	}
	convolve(holder, input, output, len);
	for(i = 0; i < len + len -1 ; i++){
		printf("convolution[%d]= %f\n", i, holder[i]);
	}

    return 0;
}

void reset(kalman_state* kinit){

    kinit->q = 0.1;
    kinit->r = 0.1;
    kinit->x = 0.39;
    kinit->p = 0.1;
    kinit->k = 0;
}

int Kalmanfilter_C(float* InputArray, float* OutputArray, kalman_state* kstate,int Length){

    int checker = 0;
    int i;
    for(i = 0; i < Length; i++){
        kstate->p = kstate->p + kstate->q;
        kstate->k = kstate->p / (kstate->p + kstate->r);
        kstate->x = kstate->x + kstate->k * (InputArray[i] - kstate->x);
        kstate->p = (1 - kstate->k) * kstate->p;

        OutputArray[i] = kstate->x;
        printf("Measured position: %f Kalman position: %f\n", InputArray[i], kstate->x);

        if(kstate->x != kstate->x){
            checker++;
        }

    }

    if(checker > 0){
        printf("Error: NaN!\n");
        return 1;
    }
    return 0;
}

void subtract(float* sub, float* in1, float* in2, int length){

    int i;
    for(i = 0; i < length; i++){
        sub[i] = in1[i] - in2[i];
    }
}

void misc(float* result, float* diff, int length){

	result[0] = mean(diff, length);
	
	float out = 0;
	int i;
	for(i = 0; i < length; i++){
		out = out + squarer(diff[i] - result[0]);
	}
	
	result[1] = root(out / length);
}

float mean(float* input, int length2){
	
	float result = 0;
	int i;
	for(i = 0; i < length2; i++){
		result = result + input[i];
	}
	return (result / length2);
}

float root(float input){
	
	int prev, k = 0;
	int kmax = 1000;
	float s = 1;
	for(k = 0; k < kmax; k++){
		prev = s;
		s = (s + input/s)/2;
		if(prev == s){
			break;
		}
	}
	return s;
}

float squarer(float input){
	
	return input * input;
}

float correlation(float* in, float* out, int length){
	
	float mean_in = mean(in, length);
	float mean_out = mean(out, length);
	float temp1 = 0; 
	float temp2 = 0;
	float temp3 = 0;
	
	int i;
	for(i = 0; i < length; i++){
		temp1 = temp1 + ((in[i] - mean_in) * (out[i] - mean_out));
		temp2 = temp2 + squarer(in[i] - mean_in);
		temp3 = temp3 + squarer(out[i] - mean_out);
	}
	
	return(temp1 / root(temp2 * temp3));
}

void convolve(float* Result, float* in1, float* in2, int length){

	int n;
	for (n = 0; n < (length + length - 1); n++){
		int kmin, kmax, k;

		Result[n] = 0;

		kmin = (n >= length - 1) ? n - (length - 1) : 0;
		kmax = (n < length - 1) ? n : length - 1;

		for (k = kmin; k <= kmax; k++){
			Result[n] += in1[k] * in2[n - k];
		}
	}
}











