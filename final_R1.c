#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <pthread.h>

#define PI 3.141592654

struct data_read{		//struct of data that is sent to PI, this corresponds to the data_send struct of Zumo
  int leftencoder;
  int rightencoder;
  int battery_mV;
};                          	// The Struct of data we would like to read from zumo, contains 12 Bytes of data;

struct data_send{
  int leftspd;
  int rightspd;
};                           	// The struct of data we send, contains 8 bytes of data;

union tx {
  struct data_send data;
  char byte_send[8];
};

union rx {
  struct data_read data;
  char byte_read[12];
};

// Union used to save different values in the same MEMORY space
// An int is 4 bytes on the pi, thus the Char must be 4 bytes large

union rx info_read;
union tx info_send;

//GLOBALS:
int file;
char *filename = "/dev/i2c-1";

FILE* plot = NULL;
char plot_name[25];

double r = 20.0/1000; //18 for tmp31
double A = 40.0/1000; //50.0
double Home_theta;
double O[2];

struct timespec program_start, res;
volatile double p[6] = {0,0,0,0,0,0};
volatile double p_ref[3] = {0,0,0};

int error_type; // 1 - straight, 0 - turn, 2 - short movement

int set_ref(double x, double y){
    p_ref[0] = x*cos(Home_theta)-(y*sin(Home_theta))+O[0];
    p_ref[1] = x*sin(Home_theta)+(y*cos(Home_theta))+O[1];
    return(0);
}
void* move(void* unused){
    //printf("Pthread Start\t");
	double run_time = 5; //Program will timeout at 5 seconds
	double Kp_X = 45;//38,35,33,38
	double Kp_R = 45;//38,35
	double Ti = 0.05;//0.025,0.0125,0.0025,0.00125;
	double Ti_R = 0.05;//0.025,0.0025;
	double Td = 0.000625;// add 0
	double p_i[3] = {0,0,0};

    double phi1, phi2, phi1_dot, phi2_dot;
    double T;
	int left_encoder[2] = {0,0};
	int right_encoder[2] = {0,0};

	double p_dot[6] = {0,0,0,0,0,0};
    double e[6] = {0,0,0,0,0,0};
    double e_int[3] = {0,0,0};
    double e_d[3] = {0,0,0};
    double E[3];

    void *b = &info_read;
    const void *a = &info_send;

	//amount of bytes actually written and read
    int read_actual, write_actual;


    int leftspd, rightspd;

    double error = 100; // Place holder
    double elapsed_time[2] = {0,0};
    double program_timestamp = 0;


    //Determine Error Type
    double error_allowable;
    if(error_type == 1){
        error_allowable = 0.01;
        printf("Driving to Point(%lf,%lf,%lf)\n", p_ref[0],p_ref[1],p_ref[2]);
    }
    if(error_type == 0){
            error_allowable = 0.005;
            printf("Rotating to Point(%lf,%lf,%lf)\n", p_ref[0],p_ref[1],p_ref[2]);
    }
    if(error_type == 2){
            error_allowable = 0.001;
            Kp_X = 50;//50
            printf("Driving to Point(%lf,%lf,%lf)\n", p_ref[0],p_ref[1],p_ref[2]);
    }
    struct timespec start_time, current_time;
    clock_gettime(CLOCK_REALTIME, &start_time);

    while((error>error_allowable)){
        //Update The Readings
	    read_actual = read(file,b,12);
	    if(read_actual !=12){
		   printf("Failed to read from the i2c Bus. read %d bytes\n",read_actual);
	    }

        left_encoder[1] = (-1*info_read.data.leftencoder);
        right_encoder[1] = (info_read.data.rightencoder);
        //encoder_correction
        if(abs(left_encoder[1]) > 1000) { left_encoder[1] = left_encoder[0];}
        if(abs(right_encoder[1]) > 1000) { right_encoder[1] = right_encoder[0];}

        // Determine Sample Time or dt for Integration and Differentiation
        T = elapsed_time[1]-elapsed_time[0];

        // Calculate phi_dot (Numerical Differentiation)
        phi1_dot = ((right_encoder[1]*2*PI/1807))/T;
        phi2_dot = ((left_encoder[1]*2*PI/1807))/T;

        if (T == 0){phi1_dot = 1; phi2_dot = 1;}

        left_encoder[0] = left_encoder[1];
        right_encoder[0] = right_encoder[1];

        // Calculate p_dot (fwd kin & Numerical Integration)

        p_dot[0] = r*(-0.5*cos(p[2])*phi1_dot+0.5*cos(p[2])*phi2_dot);
        p_dot[1] = r*(-0.5*sin(p[2])*phi1_dot+0.5*sin(p[2])*phi2_dot);
        p_dot[2] = (-1*r*(1/(2*A)))*(phi1_dot+phi2_dot);


        // Calculate p (Forward rectangular rule)
        p[0] += p_dot[3]*T;
        p[1] += p_dot[4]*T;
        p[2] += p_dot[5]*T;

        p_dot[3] = p_dot[0];
        p_dot[4] = p_dot[1];
        p_dot[5] = p_dot[2];

        //calculate error between current pose and reference pose

        e[0] = (p_ref[0]-p[0]);
        e[1] = (p_ref[1]-p[1]);
        e[2] = (p_ref[2]-p[2]);

        //e_int used for I values.
        e_int[0] += e[3]*T;
        e_int[1] += e[4]*T;
        e_int[2] += e[5]*T;

        e_d[0] = (e[0]-e[3])/T;
        e_d[1] = (e[1]-e[4])/T;
        e_d[2] = (e[2]-e[5])/T;

        e[3] = e[0];
        e[4] = e[1];
        e[5] = e[2];

        E[0] = Kp_X*(e[0] + (Ti*e_int[0])+ (Td*e_d[0]));
        E[1] = Kp_X*(e[1] + (Ti*e_int[1])+ (Td*e_d[1]));
        E[2] = Kp_R*(e[2] + (Ti_R*e_int[2]));

        //Inv Kinematics

        phi1  = (1/r)*((-1*cos(p[2])*E[0])-(sin(p[2])*E[1])-(A*E[2]));
        phi2  = (1/r)*((cos(p[2])*E[0])+(sin(p[2])*E[1])-(A*E[2]));


        leftspd =(phi2+0.717)/0.0416;//(phi2+0.717)/0.0421
        if (leftspd > 300){leftspd =300;}
        if (leftspd < -300){leftspd = -300;}
        info_send.data.leftspd =leftspd;
        rightspd =(phi1+0.665)/0.0416;
        if (rightspd >300){rightspd = 300;}
        if (rightspd <-300){rightspd = -300;}
        info_send.data.rightspd =-1*rightspd;

        //calculate error
        if(error_type==1){error = sqrt(pow(e[0],2)+pow(e[1],2));} // Straight Motion
        if(error_type==0){error = e[2];} //Rotation Motion

        if (error < 0){error = error*-1;}

        if(elapsed_time[1] >= run_time){
                info_send.data.rightspd =0;
                info_send.data.leftspd =0;
                write_actual = write(file,a,8);
                if (write_actual != 8){
                    printf("Failed to write to the i2c bus, wrote %d bytes\n", write_actual);
                }
                break;
        } // Overtime

        //Send to Robot
        write_actual = write(file,a,8);
        if (write_actual != 8){
                printf("Failed to write to the i2c bus, wrote %d bytes\n", write_actual);
        }

        elapsed_time[0] = elapsed_time[1];
        clock_gettime(CLOCK_REALTIME, &current_time);
        elapsed_time[1] = (current_time.tv_sec - start_time.tv_sec);
        elapsed_time[1] += (current_time.tv_nsec - start_time.tv_nsec)/1e9;
        program_timestamp = (current_time.tv_sec - program_start.tv_sec);
        program_timestamp += (current_time.tv_nsec - program_start.tv_nsec)/1e9;

        //print to trajectory file
        fprintf(plot,"%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%lf,%d\n",p[0],p[1],p[2],p_ref[0],p_ref[1],p_ref[2],info_read.data.leftencoder,info_read.data.rightencoder,program_timestamp,info_read.data.battery_mV);

        //delay for sampling - currently at 10 ms, allows encoders to pick up 7-8 readings.
	    usleep(10000);
	}
    info_send.data.rightspd =0;
	info_send.data.leftspd =0;
    write_actual = write(file,a,8);
    if (write_actual != 8){
            printf("Failed to write to the i2c bus, wrote %d bytes\n", write_actual);
    }
    //printf("Pthread_complete\n");
	pthread_exit(NULL);
	// Compile with $ gcc -o test1 test1.c -lm
}


int main(void){
    memset(&info_send,0,8);
    pthread_t travel;
    // Connecting to the I2C Bus on the Pi
    if ((file = open(filename, O_RDWR)) < 0){
        perror("Failed to open the i2c bus");
        exit(1);
    }

    // Connecting to the I2C Address
    int addr = 0b00001000;
    if (ioctl(file,I2C_SLAVE,addr) <0){
        printf("Failed to acqure bus access and/or talk to slave. \n");
        exit(1);
    }
	clock_getres(CLOCK_REALTIME,&res);
	printf("The clock resolution of CLOCK %d is %d and %d nsec\n",CLOCK_REALTIME,res.tv_sec,res.tv_nsec);

    //Setup CSV

    printf("Name of Trajectory File: (25 char max)\n");
    scanf("%s",&plot_name);
    plot = fopen(strcat(plot_name,".csv"),"w+");
    fprintf(plot,"X,Y,Theta,X_ref,Y_ref,Theta_ref,Left Encoder,Right Encoder,Time,Battery (mV)\n");

    clock_gettime(CLOCK_REALTIME, &program_start);

    double B[2],C[2],D[2];
	double dimension[2];
    printf("This current prototype is limited to a rectangular driveway. Please answer the following:\n");
    printf("Please enter the size of the driveway in meters: \n");
    printf("Length: "); scanf("%lf",&dimension[0]);
    printf("\nWidth: "); scanf("%lf",&dimension[1]);
	B[0] = dimension[1]; B[1] = 0;
	C[0] = 0; C[1] = dimension[0];
	D[0] = dimension[1]; D[1] = dimension[0];
	printf("\nWhat is the X and Y distance between the Robot base and Top Left corner of the driveway");
	printf("\nX: "); scanf("%lf",&O[0]);
	printf("\nY: "); scanf("%lf",&O[1]);
    printf("\nWhat is the angle between the robot base and Driveway X-Axis: "); scanf("%lf",&Home_theta);

	//robot base keeps robot at 0 rads
    p[0] = 0;
    p[1] = 0;
    p[2] = 0;

    double A_1 =0.1;


    double theta_home = atan((O[1])/(O[0]));
	//double x_transform = A*(cos(Home_theta)-sin(Home_theta))+O[0];
	//double y_transform = A*(cos(Home_theta)+sin(Home_theta))+P[1];

    //Turn to face origin
    printf("Rotating to Start Point %lf\n",theta_home);
    p_ref[0] = 0; p_ref[1] = 0;
	p_ref[2] = theta_home;
    error_type = 0;
    pthread_create (&travel,NULL,&move,NULL);
    pthread_join(travel,NULL);
    usleep(1000000);

    //Drive to Origin
    set_ref(A_1,A_1);
    //p_ref[0] = A*(cos(Home_theta)-sin(Home_theta))+O[0];
	//p_ref[1] = A*(cos(Home_theta)+sin(Home_theta))+p[1];
	p_ref[2] = p[2];
	printf("Driving to Start Point\n");
    error_type = 1;
    pthread_create(&travel,NULL,&move,NULL);
    pthread_join(travel,NULL);
    usleep(1000000);

    int n = 1;

    double y = (C[1]-A_1);
    double pi_2 = PI/2;
    while(1){ // Return to base if battery drops below 4.3V
        //STEP 1: ROTATION
        printf("Step 1: n = %d\n",n);
        p_ref[0] = p[0];
		p_ref[1] = p[1];
        p_ref[2] = pi_2+Home_theta;
        error_type = 0;
        pthread_create(&travel,NULL,&move,NULL);
        pthread_join(travel,NULL);
        usleep(1000000);
        //STEP 2: TRANSLATE
        printf("Step 2: n = %d\n",n);
        set_ref(n*A_1,y);
        //p_ref[0] = n*A*(cos(Home_theta)-sin(Home_theta))+O[0];
		//p_ref[1] = y*(cos(Home_theta)+sin(Home_theta))+p[1];
        error_type = 1;
        pthread_create(&travel,NULL,&move,NULL);
        pthread_join(travel,NULL);
        usleep(1000000);

        n++;

        //STEP 3: ROTATE
        printf("Step 3: n = %d\n",n);
        p_ref[0] = p[0];
		p_ref[1] = p[1];
        p_ref[2] = 0+Home_theta;
        error_type = 0;
        pthread_create (&travel,NULL,&move,NULL);
        pthread_join(travel,NULL);
        usleep(1000000);
        //STEP 4: TRANSLATE
        printf("Step 4: n = %d\n",n);
        set_ref(n*A_1,y);
        //p_ref[0] = n*A*(cos(Home_theta)-sin(Home_theta))+O[0];
		//p_ref[1] = y*(cos(Home_theta)+sin(Home_theta))+p[1];
        error_type = 2;
        pthread_create (&travel,NULL,&move,NULL);
        pthread_join(travel,NULL);
        usleep(1000000);
        //STEP 5: ROTATE
        printf("Step 5: n = %d\n",n);
        p_ref[0] = p[0];
		p_ref[1] = p[1];
        p_ref[2] = -1*PI/2+Home_theta;
        error_type = 0;
        pthread_create (&travel,NULL,&move,NULL);
        pthread_join(travel,NULL);
        usleep(1000000);
        //STEP 6:TRANSLATE
        printf("Step 6: n = %d\n",n);
        set_ref(n*A_1,A_1);
        //p_ref[0] = n*A*(cos(Home_theta)-sin(Home_theta))+O[0];
		//p_ref[1] = A*(cos(Home_theta)+sin(Home_theta))+p[1];
        error_type = 1;
        pthread_create (&travel,NULL,&move,NULL);
        pthread_join(travel,NULL);
        usleep(1000000);

        n++;
        if((n*A_1)>B[0]){break;}

        //STEP 7: ROTATE
        printf("Step 7: n = %d\n",n);
        p_ref[0] = p[0];
		p_ref[1] = p[1];
        p_ref[2] = 0+Home_theta;
        error_type = 0;
        pthread_create (&travel,NULL,&move,NULL);
        pthread_join(travel,NULL);
        usleep(1000000);
        //STEP 8: TRANSLATE
        printf("Step 8: n = %d\n",n);
        set_ref(n*A_1,A_1);
        //p_ref[0] = n*A*(cos(Home_theta)-sin(Home_theta))+O[0];
		//p_ref[1] = A*(cos(Home_theta)+sin(Home_theta))+p[1];
        error_type = 2;
        pthread_create (&travel,NULL,&move,NULL);
        pthread_join(travel,NULL);
        usleep(1000000);
    }
    //Turn to Origin
    printf("Turning to Origin");
    p_ref[0] = p[0];
	p_ref[1] = p[1];
    p_ref[2] = PI+Home_theta;
    error_type = 0;
    pthread_create (&travel,NULL,&move,NULL);
    pthread_join(travel,NULL);
    usleep(1000000);

    //Drive to Origin
    printf("Driving to Origin");
    set_ref(A_1,A_1);
    //p_ref[0] = A*(cos(Home_theta)-sin(Home_theta))+O[0];
	//p_ref[1] = A*(cos(Home_theta)+sin(Home_theta))+p[1];
	p_ref[2] = PI+Home_theta;
    error_type = 1;
    pthread_create (&travel,NULL,&move,NULL);
    pthread_join(travel,NULL);
    usleep(1000000);

    //Turn to Robot Base
    printf("Rotating to robot Base");
    p_ref[0] = p[0];
	p_ref[1] = p[1];
    //p_ref[2] = theta_home+PI;
    p_ref[2] = PI+atan(p[1]/p[0]);
    error_type = 0;
    pthread_create (&travel,NULL,&move,NULL);
    pthread_join(travel,NULL);
    usleep(1000000);

    //Drive to Base
    printf("Driving to Base");
    p_ref[0] = 0;
	p_ref[1] = 0;
    error_type = 1;
    pthread_create (&travel,NULL,&move,NULL);
    pthread_join(travel,NULL);
    usleep(1000000);

    //Return to initial config
    printf("Returning to original pose");
    p_ref[0] = p[0];
	p_ref[1] = p[1];
    p_ref[2] = 0;
    error_type = 0;
    pthread_create (&travel,NULL,&move,NULL);
    pthread_join(travel,NULL);
    usleep(1000000);

    pthread_exit(NULL);
}



