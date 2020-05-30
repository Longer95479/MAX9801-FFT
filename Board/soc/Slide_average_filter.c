#include "Slide_average_filter.h"
#include "include.h"

static void queue_init(float queue[], int q_length, type_complex data[])
{
	for (int i = 0; i < q_length - 1; i++)
		queue[q_length - i - 2] = data[i].re;

	queue[q_length - 1] = 0;
	
}


static float queue_in(float queue[], int q_length, float data)
{
	float out = queue[q_length - 1];
	
	for (int i = q_length -1; i > 0; i--) {
		queue[i] = queue[i - 1];
	}
	
	queue[0] = data;
	
	return out;
	
}


void slide_average_filter(type_complex sample[], int wide)
{
	float queue[(wide + 1)/2];
	
	queue_init(queue, (wide + 1)/2, sample);
	
	for (int i = (wide - 1)/2; i < (_N/2 - (wide - 1)/2); i++) {
		static float sum;
		sum = 0;
		
		for (int j = -(wide - 1)/2; j <= (wide - 1)/2; j++) {
			sum += sample[i + j].re;
		}
		
	    if (i == (wide - 1)/2)
	    	queue_in(queue, (wide + 1)/2, sum/wide);
            else
		sample[i - (wide + 1)/2].re = queue_in(queue, (wide + 1)/2, sum/wide);
	}
        
        for (int i = 0; i < (wide + 1)/2; i++)
          sample[_N/2 + i].re = 0;
}
