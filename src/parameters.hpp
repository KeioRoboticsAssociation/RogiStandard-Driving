//パラメーター

#pragma once

//定数
#define PI (3.141592653589793)
#define SQRT2 (1.414213562373095)

//固定値
#define ENC_RES_MAX (2048*2) //ロータリエンコーダーの分解能
#define TRED_RADIUS (266.0f) //中心からオムニホイールの距離[mm]
#define WHEEL_DIAMETER (60.0f)	//オムニホイールの直径	[mm]
#define WHEEL_RADIUS	(WHEEL_DIAMETER/2.0f) //オムニホイールの半径	[mm]
#define MMPP 		((WHEEL_DIAMETER*PI)/(ENC_RES_MAX))	//エンコーダ1パルスあたりに進む距離[mm]
#define RADPP 	(MMPP/TRED_RADIUS)	//エンコーダ1パルスあたりの回転角[rad]

const float KP = 1.0;
const float KI = 0.0;
const float KD = 0.0;

//モーターのPIDゲイン

#define MOTOR_0_KP_1 (1.3f)
#define MOTOR_0_KI_1 (0.06f)
#define MOTOR_0_KD_1 (0.0f)

#define MOTOR_1_KP_1 (1.3f)
#define MOTOR_1_KI_1 (0.06f)
#define MOTOR_1_KD_1 (0.0f)

#define MOTOR_2_KP_1 (1.3f)
#define MOTOR_2_KI_1 (0.06f)
#define MOTOR_2_KD_1 (0.0f)

#define MOTOR_3_KP_1 (1.3f)
#define MOTOR_3_KI_1 (0.06f)
#define MOTOR_3_KD_1 (0.0f)

#define MOTOR_0_KP_2 (0.00016f*1.17f)
#define MOTOR_0_KI_2 (0.000000f)
#define MOTOR_0_KD_2 (0.00000f)

#define MOTOR_1_KP_2 (0.00016f*1.17f)
#define MOTOR_1_KI_2 (0.00000f)
#define MOTOR_1_KD_2 (0.000000f)

#define MOTOR_2_KP_2 (0.00016f*1.17f)
#define MOTOR_2_KI_2 (0.00000f)
#define MOTOR_2_KD_2 (0.00000f)

#define MOTOR_3_KP_2 (0.00016f*1.17f)
#define MOTOR_3_KI_2 (0.000000f)
#define MOTOR_3_KD_2 (0.0f)

#define DRIVEBASE_KP (1.3f)
#define DRIVEBASE_KI (0.08f)
#define DRIVEBASE_KD (0.0f)

#define DRIVEBASE_ROTATE_KP (DRIVEBASE_KP)
#define DRIVEBASE_ROTATE_KI (DRIVEBASE_KI)
#define DRIVEBASE_ROTATE_KD (DRIVEBASE_KD)

//移動パラメータ
#define MAX_ACCELERATION (20000) //最大加速度 [mm/s^2]
#define MAX_SPEED (500) //最高速度  [mm/s]
#define MAX_ROTATE_ACCELERATION (MAX_ACCELERATION/TRED_RADIUS) //最大回転速度  [rad/s^2]
#define MAX_ROTATE_SPEED (MAX_SPEED/TRED_RADIUS) //最大回転加速度  [rad/s]

const float TARGET_VELOCITY = 2.0; // m/s
const float ACCELERATION_TIME = 3.0; // seconds

//制御周期など
#define ENCODER_LOCALIZATION_FREQUENCY (200) //エンコーダーによる自己位置推定の頻度[Hz]
#define SPEED_ADJUSTMENT_FREQUENCY (20) //速度調整の頻度[Hz]

//目的地到着を判定する閾値
#define MOVEMENT_THRESHOLD (10) //目的地に到着したとみなす半径[mm]
#define MOVEMENT_THRESHOLD_RAD (0.1f) //目的地に到着したとみなす角度の誤差[rad]


#define MOVEMENT_SWITCH_THRESHOLD (MAX_SPEED/DRIVEBASE_KP)

#define MAX_DELTA_R ((MAX_SPEED)/(SPEED_ADJUSTMENT_FREQUENCY))
#define MAX_DELTA_D ((MAX_ROTATE_SPEED)/(SPEED_ADJUSTMENT_FREQUENCY))

//フィールドパラメーター
#define SOUTH_WALL_Y (38)
#define NORTH_WALL_Y (2400-38)
#define WEST_WALL_X (38)
#define EAST_WALL_X (4500-38)

#define ROBOTSIZE (165)