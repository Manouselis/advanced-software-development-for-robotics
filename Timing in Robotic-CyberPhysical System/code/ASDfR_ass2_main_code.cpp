#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

const double LOOP_RATE = 1000.0; // Hz
const double LOOP_PERIOD = 1*(1e-3); // seconds
const int iterations_final = 1000;
void* timed_loop(void* arg) {
    double mean = 0;
    double variance = 0;
    struct timespec start_time, mid_time, end_time, sleep_time;
    double elapsed_time;
    
    int iterations = 0;
    float time_arr[iterations_final]; 
    sleep_time.tv_sec = 0;
    sleep_time.tv_nsec = 0;
    while(iterations < iterations_final) {
        clock_gettime(CLOCK_MONOTONIC, &start_time); // get start time
        // add sleep duration (in seconds)
        sleep_time.tv_sec = start_time.tv_sec;
        sleep_time.tv_nsec = start_time.tv_nsec + LOOP_PERIOD*(1e9);

        // Do some computational work here
        int j = rand() % 100;
        double x = 0.0;
        for (int k = 0; k < 10000; k++) {
            x += (double)j / (double)(k+1);
        }

        
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &sleep_time, NULL);

        clock_gettime(CLOCK_MONOTONIC, &end_time);
        time_arr[iterations] = (end_time.tv_sec - start_time.tv_sec) * 1000 + (float)(end_time.tv_nsec - start_time.tv_nsec) / (1e6); //in milliseconds
        iterations++;   
    }

    for(int i=0; i<iterations_final; i++) 
    {
    std::cout <<"time_arr[" << i << "]: " << time_arr[i] << "\n"; //print outside the while loop to avoid affecting measurements of time
    mean += time_arr[i];
    }
    mean /= iterations_final;

    for(int i=0; i<iterations_final; i++) {
        variance += pow(time_arr[i] - mean, 2);
    }
    variance /= iterations_final;

    double stddev = sqrt(variance);

    std::cout <<"Total time spend doing math in ms: " << mean*iterations_final << "\n";
    std::cout << "Mean (ms): " << mean << "\n";
    std::cout << "Standard deviation (ms): " << stddev << "\n";

    // write time_arr in txt file so we can post-process it (to make some deductions about how RT it was)
    FILE *fp;
    fp = fopen("time_arr.txt", "w");
    for (int i = 0; i < iterations_final; i++) {
        fprintf(fp, "%f\n", time_arr[i]);
    }
    fclose(fp);

    pthread_exit(NULL); // exit thread
}

int main(int argc, char** argv) {
    pthread_t thread_id;
    int ret = pthread_create(&thread_id, NULL, &timed_loop, NULL);
    // Succesfull creation of thread if ret is 0, otherwise not
    if (ret != 0) {
        fprintf(stderr, "pthread_create failed: %d\n", ret);
        exit(EXIT_FAILURE);
    }
    // Calling thread continues execution whilst the new thread runs concurrently.
    // We therefore call pthread_join to block the calling thread from coninuing until the new thread has completedstress -c 4 -m 512 -t 60
    // Wait for thread to complete
    ret = pthread_join(thread_id, NULL);
    if (ret != 0) {
        fprintf(stderr, "pthread_join failed: %d\n", ret);
        exit(EXIT_FAILURE);
    }

    
    return 0;
}