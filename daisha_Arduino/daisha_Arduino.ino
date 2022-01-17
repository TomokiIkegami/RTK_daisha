/* daisha_Arduino_v2.0 *****************************

  ・PCと磁気コンパスから、ぞれぞれずれ量と角度を取得する。
  ・取得したずれ量と角度を使用して、スタート地点から中間点へ直線走行を行い，中間点からゴール地点は45°曲がった直線経路を設定する．

****************************************************/

// モータードライバ TA8428K は前輪操舵のために使う
// バッテリーは６V使用する
// 後輪はモータードライバはリレー制御で前進のみ
// 前輪を左右に往復させロータリーエンコーダで回転角を検出する
// Rotary Encoder 読み込み：割り込み使用
// 初めにハンドルを左限界に回して enc_countA = 35　とする
// 反時計回り++   時計回り--
// 常にハンドル真ん中 enc_countA=17 で停止させる

// *******************************************************/
#include<math.h>
#include <Wire.h> //I2Cライブラリ
#include <Stepper.h>
#define MOTOR_1  (32)  // orange
#define MOTOR_2  (33)  // yellow
#define MOTOR_3  (34)  // green
#define MOTOR_4  (35)  // blue

#define Addr_Mag 0x13

#define SW  (36)
#define MOTOR_STEPS (2048) // 出力軸1回転ステップ数:2048（2相磁励）
// カタログ値です 1ステップ 0.17578度、10ステップ 1.7578度

// Relay pin //
#define RELAY1 (30)
#define RELAY2 (31)

// センサーの値を保存するグローバル関数
int   xMag;
int   yMag;
int   zMag;


const int pinA = 19;//ロータリーエンコーダA相 割り込み番号4
const int pinB = 18;//ロータリーエンコーダB相
volatile long enc_countA = 0;
const int STEER_IN1 = 7;   // 前輪操舵用
const int STEER_IN2 = 8;   // 前輪操舵用
const int STEER_IN1_B = 9;    // 前輪操舵用　予備
const int STEER_IN2_B = 10;   // 前輪操舵用　予備
const int duty0 = 0;

void compass_Rawdata();
void compass_Rawdata_Real();
double getDirection();
void stopMotor() ;

int flag = 0; //曲がり角の検出を保存するグローバル変数
int t;

// ライブラリが想定している配線が異なるので2番、3番を入れ替える
Stepper myStepper(MOTOR_STEPS, MOTOR_1, MOTOR_3, MOTOR_2, MOTOR_4);

//pinAの割り込み処理
void enc_RisingPinA()
{
  if (( digitalRead(pinA) == 0 && digitalRead(pinB) == 1 ) || ( digitalRead(pinA) == 1 && digitalRead(pinB) == 0))
    --enc_countA; // ハンドルを上から見て時計回りで--
  else if (( digitalRead(pinA) == 0 && digitalRead(pinB) == 0 ) || ( digitalRead(pinA) == 1 && digitalRead(pinB) == 1))
    ++enc_countA; // ハンドルを上から見て反時計回りで++
}

int duty_s = 255; // 前輪操舵用モータduty比 (0~255)
unsigned long tm0;
int ii = 0;

const double One_step_angle = 360.0 / MOTOR_STEPS;
const int User_steps = 57;// 回転してほしいステップ数 57で10度です
double v_angle = 0;
double cal_x = 0;
double cal_y = 0;
double cal_x_real = 0;
double cal_y_real = 0;
double ave_cal_x = 0;
double ave_cal_y = 0;
double cal_x_north = 0;
double cal_y_north = 0;
double rd = 0;
double rd_north = 0;
//double seihen = 0.16872;
//磁北の状態 旭川市春光台では西偏9度40分 = 9.667度 = 0.16872rad
// しかし電子コンパスは磁北を指さないので西偏値を引く必要は無い
int count = 0;
int rot_dir = 0;
int hensa = 0;
//ステッピングモータ回転方向 rot_dir=0 は上から見て反時計回り
//ステッピングモータ回転方向 rot_dir=1 は上から見て時計回り
//ステッピングモータ回転方向 rot_dir=4 は停止


int dest = 17;//ロータリーエンコーダとギヤ取り付け部がソフトなのでずれるが７にする 8から17.5に変更
double theta;
int delta_l;



void setup() {
  Serial.begin(115200);// arduino IDEモニタ用
  while (!Serial);
  Serial3.begin(115200);// arduino TeraTermモニタ用

  Serial.println("プログラム開始");
  Serial3.println("プログラム開始");
  xMag=yMag=zMag=1; //コンパスの値を初期化

  // RELAY Setting //
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  digitalWrite(RELAY1, 1); // 0 -> RELAY on , 1 -> RELAY off
  digitalWrite(RELAY2, 1);

  // Rotary Encoder Setting //
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  attachInterrupt(4, enc_RisingPinA, CHANGE); //両エッジで割り込み発生

  // 前輪操舵PWM用
  // Timer2(8bit timer) -> pwm 10, 9
  TCCR2B = (TCCR2B & 0b11111000) | 0x07; //30.64 [Hz]
  // Timer4(16bit timer) -> pwm 8, 7, 6
  TCCR4B = (TCCR4B & 0b11111000) | 0x05; //30.64 [Hz]

  myStepper.setSpeed(5);//5rpm ステッピングモータの回転速度
  // この回転速度で回り続けるのではなく
  // stepper.step(steps)で設定したsteps数をこの回転速度で回る
  // steps=10 ならば１０ステップをこの回転速度で回る

  //Wire(Arduino-I2C)の初期化
  Wire.begin();

  //BMX055 初期化
  BMX055_Init();
  delay(300);

  // 前輪操舵ピン設定
  pinMode(STEER_IN1, OUTPUT);
  pinMode(STEER_IN2, OUTPUT);
  analogWrite(STEER_IN1, 255);
  analogWrite(STEER_IN2, 255);
  enc_countA = 35; //14から35に変更

  while (1) {
    Serial.println(enc_countA);

    if ( enc_countA < dest) {
      // ハンドルを反時計回りに回転
      analogWrite(STEER_IN1, duty_s);
      analogWrite(STEER_IN2, duty0);
      ii = 0;
    }
    else if ( enc_countA == dest) {
      analogWrite(STEER_IN1, 255);
      analogWrite(STEER_IN2, 255);
      if (ii == 0) {
        tm0 = millis();
        ii = 1;
      }
      else {
        if ((millis() - tm0) > 3000) {
          enc_countA = 0;
          dest = 0;
          break;
        }
      }
    }
    else if ( enc_countA > dest) {
      // ハンドルを時計回りに回転
      analogWrite(STEER_IN1, duty0);
      analogWrite(STEER_IN2, duty_s);
      ii = 0;
    }
  }

  Serial.println(enc_countA);
  ii = 0;

  //ステッピングモーターを往復回転だけさせる
  while (rot_dir != 5) {
    if (rot_dir == 0) { // rot_dir=0 は上から見て反時計回り
      myStepper.step(User_steps);  // User_stepsだけ回す
      v_angle = v_angle + 10.0;
      // 電子コンパスのデータ取得
      count++;
      compass_Rawdata();
      if (v_angle >= 360.0) rot_dir = 1;
    }
    else if (rot_dir == 1) { // rot_dir=1 は上から見て時計回り
      myStepper.step(-1 * User_steps); // -1*User_stepsだけ回す
      v_angle = v_angle - 10.0;
      // 電子コンパスのデータ取得
      count++;
      compass_Rawdata();
      if (v_angle <= 0.0) rot_dir = 4;
    }
    else if (rot_dir == 2) {
      myStepper.step(User_steps);  // User_stepsだけ回す
      v_angle = v_angle + 10.0;
      // 電子コンパスのデータ取得
      count++;
      compass_Rawdata();
      if (v_angle >= 360.0) rot_dir = 3;
    }
    else if (rot_dir == 3) {
      myStepper.step(-1 * User_steps); // -1*User_stepsだけ回す
      v_angle = v_angle - 10.0;
      // 電子コンパスのデータ取得
      count++;
      compass_Rawdata();
      if (v_angle <= 0.0) rot_dir = 4;
    }
    else if (rot_dir == 4) {
      stopMotor();
      delay(500);
      rot_dir = 5;
      Serial.println("平均値");
      ave_cal_x = cal_x / count; //楕円中心のｘ座標
      Serial.println(ave_cal_x);
      ave_cal_y = cal_y / count; //楕円中心のｙ座標
      Serial.println(ave_cal_y);

      compass_Rawdata_Real();
      cal_x_north = cal_x_real - ave_cal_x;
      cal_y_north = cal_y_real - ave_cal_y;
      rd_north = getDirection(cal_x_north, cal_y_north);// y軸が角度の基準としている

      // rd_north は台車を置いた位置での磁北の方向
    }
    delay(100);
  }

  delay(400);
}

double U;   //制御量　ロータリエンコーダのパルス数
double Sum_y = 0.0; //積分要素


//フィードバック制御プログラム
int Feed_Back(double delta_rad, double delta_m) {

  // delta_rad [rad]です。  delta_m [m]です。
  
  float k[3];     //フィードバックゲイン

  double angle_num = 6; //この値を12→35→6に変更 ここの数字が大きくなると，蛇行が大きくなる。小さくし過ぎても操舵が出来なくなった。
  double DR = 12; //DR:Dual Rate ，舵角のこと。中心から片側に操舵したときに出力される、（ロータリーエンコーダの）パルスのカウント数。
  double delta_rad_2; //補正後の角度

  /////////////////////////////////////////////////////
  // 比例・積分制御                                   //
  // 進行方向の角度とずれ量の２つの比例制御 //
  //////////////////////////////////////////////////////

  k[0] = 2.0;// 進行方向の角度のゲイン (初めは2.0)
  k[1] = 50.0; // ずれ量のゲイン (初めは25.0)
  k[2] = 0.00;  // ずれ量の積分のゲイン

  if ((int)(delta_m * 100) == -128) {

    flag = 1;
    k[1] = 0; //左折時の1回のみ距離のゲインをゼロにする

  }

  delta_rad_2=delta_rad;
  
    if (flag==1){
      delta_rad_2=delta_rad-45*(PI/180); //左折をするとき、初期方位から45°（π/4）ずれるため
      Serial3.print("flag = "); Serial3.println(flag);
      }


  double hen_rad = delta_rad_2 / (PI / angle_num) * DR;


  U =  (-k[0] * hen_rad  + k[1] * delta_m + k[2] * Sum_y);  //制御量の計算
  //Serial3.println(U);
  Sum_y = delta_m + Sum_y;

  Serial3.print("d = "); Serial3.println(delta_m);
  //Serial3.print("θ1 = "); Serial3.println(delta_rad * 180 / PI);  
  Serial3.print("θ = "); Serial3.println(delta_rad_2 * 180 / PI);  
  Serial3.print("U = "); Serial3.println(U);
  Serial3.print("\n");


  if (U >= DR) U = DR;
  else if (U <= -DR) U = -DR;

  while (1) {
    if ( enc_countA < (int)U) {

      //Serial3.println("CCW"); // 反時計回り

      analogWrite(STEER_IN1, duty_s);
      analogWrite(STEER_IN2, duty0);
      Serial.print("1:enc_count,U, "); Serial.print(enc_countA); Serial.print(','); Serial.println(U);
      //ii = 0;
    }
    else if ( enc_countA == (int)U) {

      //Serial3.println("Stop!");

      analogWrite(STEER_IN1, 255);// ブレーキ
      analogWrite(STEER_IN2, 255);// ブレーキ
      Serial.print("2:enc_count,U, "); Serial.print(enc_countA); Serial.print(','); Serial.println(U);
      break;
    }
    else if ( enc_countA > (int)U) {

      //Serial3.println("CW"); // 時計回り

      analogWrite(STEER_IN1, duty0);
      analogWrite(STEER_IN2, duty_s);
      Serial.print("3:enc_count,U, "); Serial.print(enc_countA); Serial.print(','); Serial.println(U);
      //ii = 0;
    }
  }
  return (0);
}

void loop() {

  get_theta_and_d(); //車体角度:theta とずれ量:d を取得する

  ////////////////////////
  // 前輪操舵制御をする //
  ////////////////////////
  int stop_f = Feed_Back(theta, (double)delta_l / 100); //delta_l/100 → 単位を[cm]から[m]にするため．



  /*距離を受信したら走行開始*/
  digitalWrite(RELAY1, 0); // 0 -> RELAY on , 1 -> RELAY off
  digitalWrite(RELAY2, 0);

  delay(1000);

  /*一定時間走行したら停止*/
  digitalWrite(RELAY1, 1); // 0 -> RELAY on , 1 -> RELAY off
  digitalWrite(RELAY2, 1);

  if (stop_f == 1) exit(0);
  delay(300);

}

void get_theta_and_d(void) {


  ////////////////////////
  // コンパスデータ受信です //
  ////////////////////////
  compass_Rawdata_Real();
  cal_x_real = cal_x_real - ave_cal_x;
  cal_y_real = cal_y_real - ave_cal_y;
  rd = getDirection(cal_x_real, cal_y_real);

  // atan：－π/2 から π/2

  theta = rd_north - rd; //角度の値を磁気コンパスのものに置き換え
  Serial3.print("theta_real = "); Serial3.println(theta); 

  //PCからずれ量を取得

  while (1) {
    if (Serial.available() > 0) {
      byte cc = (byte)Serial.read();
      delta_l = (char)cc; //経路からのずれ量[cm]
      break;
    }

  }

}


/*磁気コンパス関連の関数*/

// コンパスの生データを取得する
void compass_Rawdata() {
  //BMX055 磁気の読み取り
  BMX055_Mag();
  Serial3.print("xMag,yMag,zMag,rot_dir,count :  ");
  Serial3.print(xMag); cal_x += (double)xMag;
  Serial3.print(",");
  Serial3.print(yMag); cal_y += (double)yMag;
  Serial3.print(",");
  Serial3.print(zMag);
  Serial3.print(",");
  Serial3.print(rot_dir);
  Serial3.print(",");
  Serial3.print(count);
  Serial3.print(",");
  Serial3.println();
  if (xMag==0&&yMag==0&&zMag==0){
    Serial3.print("compass value error!");
    xMag=yMag=zMag=1;
    delay(100);
    exit(0);
    }
}

void compass_Rawdata_Real() {
  //BMX055 磁気の読み取り
  BMX055_Mag();
  cal_x_real = (double)xMag;
  cal_y_real = (double)yMag;
  if (xMag==0&&yMag==0){
    Serial3.print("compass value error!");
    xMag=yMag=zMag=1;
    delay(100);
    exit(0);
    }
}

//** 角度を求める 出力は Radian **//
double getDirection(double x, double y) {
  double dir;
  if (y == 0) dir = 0;
  else if (x == 0 && y > 0) dir = PI / 2.0;
  else if (x == 0 && y < 0) dir = -PI / 2.0;
  else {
    dir = atan(y / x); // 戻り値 -pi/2 から +pi/2
    if (x < 0 && y >= 0) dir = dir + PI;
    else if (x < 0 && y <= 0) dir = dir - PI;
  }
  // 結果として　-PI < dir < PI
  return dir;
}


//=====================================================================================//
void BMX055_Init()
{
  //BMX055 初期化
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x0F);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}

//=====================================================================================//
void BMX055_Mag()
{
  //BMX055 磁気の読み取り
  unsigned int data[6];

  for (int i = 0; i < 6; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr_Mag);
    // Select data register
    Wire.write((66 + i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(Addr_Mag, 1);

    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
    {
      data[i] = Wire.read();
    }
  }

  // Convert the data
  xMag = ((data[1] * 256) + (data[0] & 0xF8)) / 8;
  if (xMag > 4095)
  {
    xMag -= 8192;
  }
  yMag = ((data[3] * 256) + (data[2] & 0xF8)) / 8;
  if (yMag > 4095)
  {
    yMag -= 8192;
  }
  zMag = ((data[5] * 256) + (data[4] & 0xFE)) / 2;
  if (zMag > 16383)
  {
    zMag -= 32768;
  }
}


// モーターへの電流を止める
void stopMotor() {
  digitalWrite(MOTOR_1, LOW);
  digitalWrite(MOTOR_2, LOW);
  digitalWrite(MOTOR_3, LOW);
  digitalWrite(MOTOR_4, LOW);
}
