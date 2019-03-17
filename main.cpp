#include "mbed.h"
#include "math.h"
#include "SDFileSystem.h"
#include <VL6180x.h>
#define WHO_AM_I 0x75
#define VL6180X_ADDRESS 0x29
//setup

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
char readByte(uint8_t address, uint8_t subAddress);
void initial(float initial[14]);
void write_buffer(float initial[14],float buffer[14]);
void fl(float initial[14],float pre_buffer[14],float pre2_buffer[14],float pre3_buffer[14],float pre4_buffer[14],float r_buffer[14]);//do
void printIdentification(struct VL6180xIdentification *temp);//setup VL6180X 


Serial pc(USBTX,USBRX);
SDFileSystem sd(p5,p6,p7,p8,"sd");
//LocalFileSystem local("local");
VL6180x sensor(p9, p10, VL6180X_ADDRESS<<1);// mbed uses 8bit addresses shift address by 1 bit left
//DigitalOut ch(p11);
AnalogIn potentio(p15);  //knee angle degree
AnalogIn hl_1(p17);//front pressure ofHAL shoes
AnalogIn hl_2(p18);//back pressure of HAL shoes
PwmOut  buz(p25);//buzzer 
I2C i2c(p28,p27);//IMU
DigitalOut led1(LED1);
DigitalOut led2(LED2);
Ticker flipper;
FILE *fp;

char rawdat[14];
uint8_t rawdata[14];
int16_t ac_ofs[3];//offset of acceleration
int16_t gy_ofs[3];//offset of gyro
char buf[448];
float pr1_buf[14];
float pr2_buf[14];
float pr3_buf[14];
float pr4_buf[14];
float r_buf[14];
int knee_angle = 0;//dog
DigitalIn dir(p11);
InterruptIn motor_bls(p12);//dog


VL6180xIdentification identification;
const int addr = 0x69 << 1;//スレーブアドレスslave address of IMU
const int adrAK = 0x0C << 1;//コンパス(AK)スレーブアドレス 　slave address of terrestrial magnetism
//uint8_t whoami;
//uint8_t ID;
Timer t;
Timer tl;

void motor_cnt(){//dog
    if(dir.read()==1){
        knee_angle++;
    }else{
        knee_angle--;
    }
}


int main() {
    pc.baud(115200);   
//    ch = 0;dog
    motor_bls.rise(&motor_cnt);//dog
    motor_bls.fall(&motor_cnt);//dog
    wait(2); 
    sensor.getIdentification(&identification); // Retrieve manufacture info from device memory
    //printIdentification(&identification); // Helper function to print all the Module information
    
   if(sensor.VL6180xInit() != 0){
       printf("FAILED TO INITALIZE\n"); //Initialize device and check for errors
    }; 
    sensor.VL6180xDefautSettings(); //Load default settings to get started.
//    fp = fopen("/local/out.dat", "wb");
    fp = fopen("/sd/test.dat", "wb");
    if(fp == NULL) {
        error("Could not open file for write\n");
        buz = 0.7;
    }
    setvbuf(fp,buf,_IOFBF,sizeof(buf));
    float in[14] = {1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    //whoami = readByte(addr,0x75);
    //ID = readByte(adrAK,0x00);
    char data[1]; // `data` will store the register data     
    char data_write[1];
    data_write[0] = 0x75;
    
    i2c.write(addr, data_write, 1, 1); // no stop
    i2c.read(addr, data, 1, 0); 
    //whoami = data[0];
    //setup
    //pc.printf("whoami 0x%x\n\r",whoami);
    //pc.printf("ID 0x%x\n\r",ID);
    wait(3);
    writeByte(addr,0x6B,0x00);
    writeByte(addr,0x37,0x02);
    writeByte(addr,0x28,0x00);
    writeByte(adrAK,0x0A,0x12);
   
    //read offset value of three accelerations
    data_write[0] = 0x77;
    i2c.write(addr, data_write, 1, 1); // no stop
    i2c.read(addr, rawdat, 6, 0); 
    for(int i = 0;i < 6;i++){
        rawdata[i] = rawdat[i];
    }    
    ac_ofs[0] = (int16_t)(((int16_t)rawdata[0] << 8) | rawdata[1]);
    ac_ofs[1] = (int16_t)(((int16_t)rawdata[2] << 8) | rawdata[3]);
    ac_ofs[2] = (int16_t)(((int16_t)rawdata[4] << 8) | rawdata[5]);
    //read offset value of three gyro
    data_write[0] = 0x13;
    i2c.write(addr, data_write, 1, 1); // no stop
    i2c.read(addr, rawdat, 6, 0); 
    for(int i = 0;i < 6;i++){
        rawdata[i] = rawdat[i];
    }    
    gy_ofs[0] = (int16_t)(((int16_t)rawdata[0] << 8) | rawdata[1]);
    gy_ofs[1] = (int16_t)(((int16_t)rawdata[2] << 8) | rawdata[3]);
    gy_ofs[2] = (int16_t)(((int16_t)rawdata[4] << 8) | rawdata[5]); 
    t.start();
    //initialize
    //ジャイロローパス,計測範囲変更
    writeByte(addr,0x6b,0x80);
    writeByte(addr,0x6b,0x00);
    writeByte(addr,0x1a,0x01);
    writeByte(addr,0x1b,0x19);//1000dps
    
    //
    writeByte(addr,0x1c,0x00);//加速度計測範囲変更 (+-2g)///角度推定用/https://os.mbed.com/users/Gaku0606/code/mpu9250_i2c/docs/fad6675651c1/mpu9250__i2c_8cpp_source.html
    writeByte(addr,0x1d,0x11);//加速度ローパス
    int x = 0;
    while(t.read() < 5){
        initial(in);
        x++;
    }
    for(int i = 1;i < 11;i++){
        in[i] = in[i] / (float)x;
   }
    in[11] = atan2(in[1],in[2]);
    in[12] = atan2(in[3],in[2]);
    
    
    buz = 0.7;
    wait(0.5);
    buz = 0;
    led1 = 1;
    t.reset();
    tl.start();
    write_buffer(in,pr1_buf);
    write_buffer(in,pr2_buf);
    write_buffer(in,pr3_buf);
    write_buffer(in,pr4_buf);
//    ch = 1;dog
    
   // wait(6);
    //ch = 0;
    
    while(t.read() < 15){
         pr3_buf[1] = t.read();
         fl(in,pr1_buf,pr2_buf,pr3_buf,pr4_buf,r_buf);
         while(1){
              if(tl.read() >= 0.002)break;
          }
         tl.reset();
           pr3_buf[0] = t.read();
    }
    //end
    led2 = 1;   
    
    buz = 0.7;
    wait(0.5);
    buz = 0;
     float fin[14] ={-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    fwrite(fin,sizeof(fin),1,fp);
    fclose(fp);
//    ch = 0;dog
}
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data){
    char data_write[2];
    data_write[0] = subAddress;
    data_write[1] = data;
    i2c.write(address, data_write, 2, 0);
} 
char readByte(uint8_t address, uint8_t subAddress){
    char data[1]; // `data` will store the register data     
    char data_write[1];
    data_write[0] = subAddress;
    i2c.write(address, data_write, 1, 1); // no stop
    i2c.read(address, data, 1, 0); 
    return data[0]; 
}
void read_IMU(int16_t ac_data[7]){
    char data_write[1];
    data_write[0] = 0x3B;
    i2c.write(addr, data_write, 1, 1); // no stop
    i2c.read(addr, rawdat, 14, 0); 
//for 文遅れなし
    for(int i = 0;i < 14;i++){
        rawdata[i] = rawdat[i];
    }
    for(int i = 0;i < 7;i++){
        ac_data[i] = (int16_t)(((int16_t)rawdata[2 * i] << 8) | rawdata[2 * i + 1]);
        }      
    }
void initial(float initial[14]){
    int16_t ac_data[7];
    read_IMU(ac_data);
    initial[1] = initial[1] - (float)(ac_data[0] - ac_ofs[0]) * 0.11889;
    initial[2] = initial[2] + (float)(ac_data[1] - ac_ofs[1]) * 0.12342;
    initial[3] = initial[3] + (float)(ac_data[2] - ac_ofs[2]) * 0.122;
//ジャイロ       
    initial[4] = initial[4] + (float)(ac_data[4] - gy_ofs[0]) * 0.00763;//スケールファクタ3.3
    initial[5] = initial[5] + (float)(ac_data[5] - gy_ofs[1]) * 0.00763;
    initial[6] = initial[6] + (float)(ac_data[6] - gy_ofs[2]) * 0.008513;//
    initial[7] = initial[7] + hl_1.read();
    initial[8] = initial[8] + hl_2.read();
    initial[9] = initial[9] + (float)sensor.getDistance();
    float knee_angle;
    knee_angle = potentio * (-333.3);
    initial[10] = initial[10] +  knee_angle;
//    initial[10]=0;
    //fwrite(in,sizeof(in),1,fp);
}

void write_buffer(float initial[14],float buffer[14]){
    int16_t ac_data[7];
    read_IMU(ac_data);
//for 文遅れなし
    buffer[1] =   - 1 * (((float)(ac_data[0] - ac_ofs[0]) * 0.122) + initial[1]);// 2/0x8000 = 0.061035
    buffer[2] = (((float)(ac_data[1] - ac_ofs[1]) * 0.122) - (initial[2] - 1000));// 8/0x8000 = 0.244///
    buffer[3] =   (float)(ac_data[2] - ac_ofs[2])* 0.122 - initial[3];
//ジャイロ   
    buffer[13] = ac_data[3];
    buffer[4] = (float)(ac_data[4] - gy_ofs[0]) * 0.00763 - initial[4];//
    buffer[5] =  (float)(ac_data[5] - gy_ofs[1]) * 0.00763 - initial[5];//
    buffer[6] =(float)(ac_data[6] - gy_ofs[2]) * 0.008518 - initial[6];
    buffer[7] =  hl_1.read();
    buffer[8] = hl_2.read();
    buffer[9] = (float)sensor.getDistance();
  //  float knee_angle;
//    knee_angle = potentio * (-333.3);
       
    buffer[10] = float(knee_angle);//dog - initial[10];//  pc.printf("%f %f\n",Knee_angle,buffer[10]);
    buffer[0] = t.read();
    }
    
    
   
void fl(float initial[14],float pre_buffer[14],float pre2_buffer[14],float pre3_buffer[14],float pre4_buffer[14],float r_buffer[14]){
    float buffer[14];
    /*Timer df;
    df.start();
    for(int i = 0;i < 9;i++){
        write_buffer(initial,buffer);
        while(1){
              if(df.read() >= 0.001)break;
          }
         df.reset(); 
    }
    write_buffer(initial,buffer);*/
    write_buffer(initial,buffer);
    pre3_buffer[2] = t.read();
    /*for(int i = 1;i < 11;i++){
        buffer[i] = buffer[i] / 10.0;
    }*/
    float write_buf[14];
    for(int i = 1;i < 7;i++){
        write_buf[i] = buffer[i];// + pre_buffer[i] + pre2_buffer[i] + pre3_buffer[i] + pre4_buffer[i]) / 5.0;
        pre_buffer[i] = buffer[i];
        pre2_buffer[i] = pre_buffer[i];
        pre3_buffer[i] = pre2_buffer[i];
        pre4_buffer[i] = pre3_buffer[i];
       }
       pre3_buffer[3] = t.read();
    for(int i = 7;i < 11;i++){
        write_buf[i] = buffer[i];
        pre_buffer[i] = buffer[i];
        pre2_buffer[i] = pre_buffer[i];
      //  pre3_buffer[i] = pre2_buffer[i];
     //   pre4_buffer[i] = pre3_buffer[i];
        }
        pre3_buffer[4] = t.read();
        write_buf[13] =(buffer[13] / 333.87) + 21.0;
    float k1,k2;
    float x_buf,y_buf,z_buf;
    x_buf = (write_buf[1] + pre_buffer[1] + pre2_buffer[1]) / 3;
    y_buf = (write_buf[2] + pre_buffer[2] + pre2_buffer[2]) / 3;
    z_buf = (write_buf[3] + pre_buffer[3] + pre2_buffer[3]) / 3;
    
    k1 = 0.05 * 1000 / (1000 + fabs(1000 - sqrt(write_buf[1] *write_buf[1] + write_buf[2] * write_buf[2])));//30000
    k2 = 0.05 * 1000 / (1000 + fabs(1000 - sqrt(write_buf[2] * write_buf[2] + write_buf[3] * write_buf[3])));
    write_buf[0] = t.read();
    write_buf[11] =   (1 - k1) * (r_buffer[11] + write_buf[6] *(buffer[0] - r_buffer[0])) +  k1 * atan2(x_buf,y_buf) * 180 / 3.14;//angle
    write_buf[12] = /*(1 - k2) * (r_buffer[12]+ write_buf[4] *(buffer[0] - r_buffer[0])) + k2 **/ atan2(z_buf,y_buf) * 180 / 3.14;
    if(write_buf[9] > 220) write_buf[9] = 250;
    else write_buf[9] = write_buf[9];
    //write_buf[0] = t.read();
    pre3_buffer[5] = t.read();
    fwrite(write_buf,sizeof(write_buf),1,fp);
    //pc.printf("%f  %f,%f,  %f\n",write_buf[10],write_buf[1],write_buf[2],atan2(write_buf[1],write_buf[2]) * 180 / 3.14);
    r_buffer[11] = write_buf[11];
    r_buffer[12] = write_buf[12];
    write_buf[0] = t.read();
    for(int i = 0;i < 14;i++){
        r_buffer[i] = write_buf[i];
        }
}

    void printIdentification(struct VL6180xIdentification *temp){
    printf("Model ID = ");
    printf("%d\n",temp->idModel);
    printf("Model Rev = ");
    printf("%d",temp->idModelRevMajor);
    printf(".");
    printf("%d\n",temp->idModelRevMinor);
    printf("Module Rev = ");
    printf("%d",temp->idModuleRevMajor);
    printf(".");
    printf("%d\n",temp->idModuleRevMinor);  
    printf("Manufacture Date = ");
    printf("%d",((temp->idDate >> 3) & 0x001F));
    printf("/");
    printf("%d",((temp->idDate >> 8) & 0x000F));
    printf("/1");
    printf("%d\n",((temp->idDate >> 12) & 0x000F));
    printf(" Phase: ");
    printf("%d\n",(temp->idDate & 0x0007));
    printf("Manufacture Time (s)= ");
    printf("%d\n",(temp->idTime * 2));
    printf("\n\n");
}