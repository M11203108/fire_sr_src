#include "zlac8015d.h"

int main(){
    ZLAC mot;
    ZLAC mot1;
    struct MOT_DATA motorstat;
    mot.init("/dev/motorttyUSB0", 115200, 0x01, true);
    mot1.init("/dev/motorttyUSB1", 115200, 0x01, true);
    
    mot.set_double_rpm(0, 0);
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_position();
    printf("\nR:%lf|L:%lf\n", motorstat.rpm_R, motorstat.rpm_L);
    printf("\nRp:%d|Lp:%d\n", motorstat.encoder_R, motorstat.encoder_L);
        mot.sleep(1000);

    printf("===set_rpm===\n");
    mot.set_double_rpm(5, 0);   
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_position();
    printf("\nR:%lf|L:%lf\n", motorstat.rpm_R, motorstat.rpm_L);
    printf("\nRp:%d|Lp:%d\n", motorstat.encoder_R, motorstat.encoder_L);
        mot.sleep(1000);

    printf("===set_rpm===\n");
    mot.set_double_rpm(10, 0);   
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_position();
    printf("\nR:%lf|L:%lf\n", motorstat.rpm_R, motorstat.rpm_L);
    printf("\nRp:%d|Lp:%d\n", motorstat.encoder_R, motorstat.encoder_L);
        mot.sleep(1000);

    mot.set_double_rpm(0, 0); 
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_position();
    printf("\nR:%lf|L:%lf\n", motorstat.rpm_R, motorstat.rpm_L);
    printf("\nRp:%d|Lp:%d\n", motorstat.encoder_R, motorstat.encoder_L);

    printf("===disable===\n");
    mot.terminate();
    // motorL.disable();
}
