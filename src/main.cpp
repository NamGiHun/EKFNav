#include "Serial.h"
#include "HgDataParser.h"

int main()
{
	HgDataParser IMU_Class;
	printf("\nRateX\t\tRateY\t\tRateZ\t\tAccelX\t\tAccelY\t\tAccelZ\t\tTemp\t#Chks Err.");
	printf("\n-----------------------------------------------------------------------------------------------------------------\n");
	IMU_Class.ReadPort();

	while (1){
		IMU_Class.ReadIMU();
		printf("%f %f %f %f %f %f\n", IMU_Class.IMU[0], IMU_Class.IMU[1], IMU_Class.IMU[2], IMU_Class.IMU[3], IMU_Class.IMU[4], IMU_Class.IMU[5]);
	}
	return 0;
}

