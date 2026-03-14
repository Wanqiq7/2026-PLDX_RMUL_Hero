/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 鍙互鐢?pragma once浠ｆ浛#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "math.h"
#include "stdint.h"
#include "vision_comm.h"

/* 寮€鍙戞澘绫诲瀷瀹氫箟,鐑у綍鏃舵敞鎰忎笉瑕佸紕閿欏搴斿姛鑳?淇敼瀹氫箟鍚庨渶瑕侀噸鏂扮紪璇?鍙兘瀛樺湪涓€涓畾涔?
 */
// #define ONE_BOARD // 鍗曟澘鎺у埗鏁磋溅
// #define CHASSIS_BOARD // 搴曠洏鏉?
#define GIMBAL_BOARD // 浜戝彴鏉?

/* -------------------------瑙嗚閫氫俊閾捐矾閫夋嫨------------------------- */
/**
 * @brief 瑙嗚閾捐矾閫夋嫨寮€鍏筹紙缂栬瘧鏈燂級
 * @note 榛樿浣跨敤 CAN锛屽闇€鏀瑰洖 USB 铏氭嫙涓插彛锛圴CP锛夛紝璇峰皢 VISION_LINK_TYPE 璁句负
 * VISION_LINK_VCP銆?CAN 鍗忚瀵归綈 sp_vision_25-main 鐨?
 * io/cboard.cpp锛坬uaternion_canid / bullet_speed_canid / send_canid锛夈€?
 */
#define VISION_LINK_VCP 0
#define VISION_LINK_CAN 1

#ifndef VISION_LINK_TYPE
#define VISION_LINK_TYPE VISION_LINK_CAN
#endif

/**
 * @brief CAN 瑙嗚閾捐矾鍙傛暟锛堜粎鍦?VISION_LINK_TYPE==VISION_LINK_CAN 鏃剁敓鏁堬級
 * @note CAN
 * 鍙傛暟宸蹭笅娌夊埌瑙嗚閫氫俊妯″潡锛坄modules/vision_comm/vision_comm.h`锛変腑缁熶竴绠＄悊銆?
 */
/* 鏈哄櫒浜洪噸瑕佸弬鏁板畾涔?娉ㄦ剰鏍规嵁涓嶅悓鏈哄櫒浜鸿繘琛屼慨鏀?娴偣鏁伴渶瑕佷互.0鎴杅缁撳熬,鏃犵鍙蜂互u缁撳熬
 */
// 鎺у埗鍛ㄦ湡鍙傛暟
#define ROBOT_CTRL_PERIOD_S 0.001f  // 涓绘帶鍒跺懆鏈?[s]锛屽搴?kHz
#define VISION_CTRL_PERIOD_S 0.001f // 瑙嗚鎺у埗鍛ㄦ湡 [s]锛屼笌涓绘帶鍒跺悓姝?

// 浜戝彴鍙傛暟
#define YAW_CHASSIS_ALIGN_ECD                                                  \
  5695 // 浜戝彴鍜屽簳鐩樺榻愭寚鍚戠浉鍚屾柟鍚戞椂鐨勭數鏈虹紪鐮佸櫒鍊?鑻ュ浜戝彴鏈夋満姊版敼鍔ㄩ渶瑕佷慨鏀?
#define PITCH_MAX_ANGLE                                                        \
  39.5f // 浜戝彴绔栫洿鏂瑰悜鏈€澶ц搴?(娉ㄦ剰鍙嶉濡傛灉鏄檧铻轰华锛屽垯濉啓闄€铻轰华鐨勮搴?
#define PITCH_MIN_ANGLE                                                        \
  -10.5f // 浜戝彴绔栫洿鏂瑰悜鏈€灏忚搴?(娉ㄦ剰鍙嶉濡傛灉鏄檧铻轰华锛屽垯濉啓闄€铻轰华鐨勮搴?
// Pitch 閲嶅姏琛ュ伩鍙傛暟锛堝弬鑰?derivation_process.md 2.2锛?
// k = m * g * r锛屽崟浣?N路m锛実amma 涓洪噸蹇冪浉瀵规灙绾跨殑鍋忕疆瑙掞紙搴︼級
#define PITCH_GRAVITY_K 0.0f
#define PITCH_GRAVITY_GAMMA_DEG 0.0f
// 鍙戝皠鍙傛暟
#define REDUCTION_RATIO_LOADER 51.0f // M3508鎷ㄧ洏鐢垫満鍑忛€熺鍑忛€熸瘮锛?1:1锛?
#define LOAD_ANGLE_PER_BULLET 60     // 鎷ㄧ洏杈撳嚭杞存瘡鍙戝脊涓歌浆鍔ㄨ搴︼紙鏈烘璁捐鍊硷級
#define ONE_BULLET_DELTA_ANGLE                                                 \
  (LOAD_ANGLE_PER_BULLET *                                                     \
   REDUCTION_RATIO_LOADER) // 鐢垫満杞磋搴?= 杈撳嚭杞磋搴?脳 鍑忛€熸瘮 = 60脳51 = 3060掳
#define NUM_PER_CIRCLE 6   // 鎷ㄧ洏涓€鍦堢殑瑁呰浇閲?
// 鐑噺鍓嶉鍙傛暟锛堝崟浣嶏細d[heat/鍙慮锛宼[s]锛宻hoot_rate[鍙?s]锛?
#define HEAT_PER_SHOT_D 100.0f
#define FEEDFORWARD_T_TARGET_S 1.0f
#define SHOOT_RATE_MIN 0.5f
#define SHOOT_RATE_MAX 3.0f
#define SHOOT_RATE_SAFE 1.0f
#define SHOOT_FIXED_BULLET_SPEED SMALL_AMU_12
// 鏈哄櫒浜哄簳鐩樹慨鏀圭殑鍙傛暟,鍗曚綅涓簃m(姣背)
#define WHEEL_BASE 560  // 绾靛悜杞磋窛(鍓嶈繘鍚庨€€鏂瑰悜)
#define TRACK_WIDTH 330 // 妯悜杞窛(宸﹀彸骞崇Щ鏂瑰悜)
#define CENTER_GIMBAL_OFFSET_X                                                 \
  0 // 浜戝彴鏃嬭浆涓績璺濆簳鐩樺嚑浣曚腑蹇冪殑璺濈,鍓嶅悗鏂瑰悜,浜戝彴浣嶄簬姝ｄ腑蹇冩椂榛樿璁句负0
#define CENTER_GIMBAL_OFFSET_Y                                                 \
  0 // 浜戝彴鏃嬭浆涓績璺濆簳鐩樺嚑浣曚腑蹇冪殑璺濈,宸﹀彸鏂瑰悜,浜戝彴浣嶄簬姝ｄ腑蹇冩椂榛樿璁句负0
#define RADIUS_WHEEL 0.077f // 杞瓙鍗婂緞(鍗曚綅m,娉ㄦ剰涓嶆槸鐩村緞)
#define REDUCTION_RATIO_WHEEL                                                  \
  19.0f // 鐢垫満鍑忛€熸瘮,鍥犱负缂栫爜鍣ㄩ噺娴嬬殑鏄浆瀛愮殑閫熷害鑰屼笉鏄緭鍑鸿酱鐨勯€熷害鏁呴渶杩涜杞崲
#define CHASSIS_MASS 17.0f     // 鏈哄櫒浜烘暣澶囪川閲?鍗曚綅kg,鐢ㄤ簬鍔熺巼璁＄畻
#define GRAVITY_ACCEL 9.81f    // 閲嶅姏鍔犻€熷害,鍗曚綅m/s^2,鐢ㄤ簬鍔熺巼璁＄畻
#define DIST_CG_FRONT_AXLE 280 // 閲嶅績璺濆墠杞磋窛绂?鍗曚綅mm
#define DIST_CG_REAR_AXLE 280  // 閲嶅績璺濆悗杞磋窛绂?鍗曚綅mm
#define CG_HEIGHT 132          // 閲嶅績璺濆簳鐩樹腑蹇冮珮搴?鍗曚綅mm
// 搴曠洏璺熼殢灏辫繎鍥炰腑鍙傛暟
#define CHASSIS_FOLLOW_ALLOW_FLIP 1 // 鏄惁鍏佽杞﹀ご缈昏浆(0:涓嶅厑璁? 1:鍏佽)
#define CHASSIS_FOLLOW_FLIP_THRESHOLD 90.0f // 杞﹀ご缈昏浆瑙﹀彂闃堝€?搴?
#define CHASSIS_FOLLOW_MAX_ERR 135.0f       // 鏈€澶у厑璁歌宸?搴?,閬垮厤鎺у埗閲忚繃澶?
// 閿洏鎺у埗鐩稿叧鍙傛暟
//  褰撳墠閿洏璺緞鐩存帴浣跨敤褰掍竴鍖栫洰鏍囧€糩-1, 1]锛?//  鍥犳鏂滃潯鍙傛暟涔熷繀椤讳娇鐢ㄢ€滃綊涓€鍖栧€?绉掆€濋噺绾层€?// 鍔犻€熷害 (鍗曚綅: 褰掍竴鍖栧€?绉?
// dt=5ms 鏃讹紝姣忓懆鏈熺害澧炲姞 0.01锛?->1 绾﹂渶 0.5s
#define KEYBOARD_RAMP_ACCEL 2.0f
// 鍑忛€熷害 (鍗曚綅: 褰掍竴鍖栧€?绉?
// dt=5ms 鏃讹紝姣忓懆鏈熺害鍑忓皬 0.015锛屾澗鎵嬪洖闆剁害闇€ 0.33s
#define KEYBOARD_RAMP_DECEL 3.0f
// 鍙嶅悜鍒跺姩鍑忛€熷害 (鍗曚綅: 褰掍竴鍖栧€?绉?
// dt=5ms 鏃讹紝姣忓懆鏈熺害鍙樺寲 0.02锛屾崲鍚戣繃闆剁害闇€ 0.25s
#define KEYBOARD_RAMP_BRAKE_DECEL 4.0f
/**
 * @brief M3508鐢垫満鎵煩鍒癈AN鎸囦护鍊肩殑杞崲绯绘暟,1N路m瀵瑰簲2730.67鐨凜AN鎸囦护鍊?
 * @note 鏍规嵁瀹樻柟鏁版嵁锛氶瀹氭壄鐭?N路m @ 10A鐢垫祦, C620鐢佃皟 -20A~20A 瀵瑰簲
 * -16384~16384                                璁＄畻鍏紡: (10A / 20A * 16384) /
 * 3N路m = 8192 / 3 鈮?2730.67
 */
#define M3508_TORQUE_TO_CURRENT_CMD_COEFF 2730.67f

#define GYRO2GIMBAL_DIR_YAW                                                    \
  1 // 闄€铻轰华鏁版嵁鐩歌緝浜庝簯鍙扮殑yaw鐨勬柟鍚?1涓虹浉鍚?-1涓虹浉鍙?
#define GYRO2GIMBAL_DIR_PITCH                                                  \
  1 // 闄€铻轰华鏁版嵁鐩歌緝浜庝簯鍙扮殑pitch鐨勬柟鍚?1涓虹浉鍚?-1涓虹浉鍙?
#define GYRO2GIMBAL_DIR_ROLL                                                   \
  1 // 闄€铻轰华鏁版嵁鐩歌緝浜庝簯鍙扮殑roll鐨勬柟鍚?1涓虹浉鍚?-1涓虹浉鍙?

// 妫€鏌ユ槸鍚﹀嚭鐜颁富鎺ф澘瀹氫箟鍐茬獊,鍙厑璁镐竴涓紑鍙戞澘瀹氫箟瀛樺湪,鍚﹀垯缂栬瘧浼氳嚜鍔ㄦ姤閿?
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) ||                          \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||                           \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 鍘嬬缉缁撴瀯浣?鍙栨秷瀛楄妭瀵归綈,涓嬮潰鐨勬暟鎹兘鍙兘琚紶杈?
/* -------------------------鍩烘湰鎺у埗妯″紡鍜屾暟鎹被鍨嬪畾涔?------------------------*/
/**
 * @brief 杩欎簺鏋氫妇绫诲瀷鍜岀粨鏋勪綋浼氫綔涓篊MD鎺у埗鏁版嵁鍜屽悇搴旂敤鐨勫弽棣堟暟鎹殑涓€閮ㄥ垎
 *
 */
// 鏈哄櫒浜虹姸鎬?
typedef enum {
  ROBOT_STOP = 0,
  ROBOT_READY,
} Robot_Status_e;

// 搴旂敤鐘舵€?
typedef enum {
  APP_OFFLINE = 0,
  APP_ONLINE,
  APP_ERROR,
} App_Status_e;

// 搴曠洏妯″紡璁剧疆
/**
 * @brief 鍚庣画鑰冭檻淇敼涓轰簯鍙拌窡闅忓簳鐩?鑰屼笉鏄搴曠洏鍘昏拷浜戝彴,浜戝彴鐨勬儻閲忔瘮搴曠洏灏?
 *
 */
typedef enum {
  CHASSIS_ZERO_FORCE = 0,    // 鐢垫祦闆惰緭鍏?
  CHASSIS_ROTATE,            // 灏忛檧铻烘ā寮?
  CHASSIS_NO_FOLLOW,         // 涓嶈窡闅忥紝鍏佽鍏ㄥ悜骞崇Щ
  CHASSIS_FOLLOW_GIMBAL_YAW, // 璺熼殢妯″紡锛屽簳鐩樺彔鍔犺搴︾幆鎺у埗
} chassis_mode_e;

// 浜戝彴妯″紡璁剧疆
typedef enum {
  GIMBAL_ZERO_FORCE = 0, // 电流零输出
  GIMBAL_GYRO_MODE = 2,  // 云台陀螺仪反馈模式，保留原有协议数值
  GIMBAL_AUTOAIM_MODE = 3, // 自瞄模式：Yaw 视觉双环（直出电流），Pitch 视觉参考限速
  GIMBAL_LQR_MODE = 4,     // 云台 LQR 控制模式
  GIMBAL_SYS_ID_CHIRP = 5, // 云台正弦扫频辨识模式
} gimbal_mode_e;

// 鍙戝皠妯″紡璁剧疆
typedef enum {
  SHOOT_OFF = 0,
  SHOOT_ON,
} shoot_mode_e;
typedef enum {
  FRICTION_OFF = 0, // 摩擦轮关闭
  FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum {
  LOAD_STOP = 0,  // 鍋滄鍙戝皠
  LOAD_REVERSE,   // 鍙嶈浆
  LOAD_1_BULLET,  // 鍗曞彂
  LOAD_3_BULLET,  // 涓夊彂
  LOAD_BURSTFIRE, // 杩炲彂
} loader_mode_e;

// 瑙嗚鎺у埗妯″紡璁剧疆
typedef enum {
  VISION_MODE_OFF = 0,        // 瑙嗚鍏抽棴
  VISION_MODE_AUTO_AIM = 1,   // 鑷姩鐬勫噯
  VISION_MODE_SMALL_BUFF = 2, // 灏忕
  VISION_MODE_BIG_BUFF = 3,   // 澶х
  VISION_MODE_ENERGY_HIT =
      VISION_MODE_SMALL_BUFF, // 鍏煎鏃ц兘閲忔満鍏虫灇涓撅紝鏄犲皠涓哄皬绗?
  VISION_MODE_MANUAL_AIM = 4, // 鎵嬪姩杈呭姪鐬勫噯
} vision_mode_e;

// 鍔熺巼闄愬埗,浠庤鍒ょ郴缁熻幏鍙?鏄惁鏈夊繀瑕佷繚鐣?
typedef struct { // 鍔熺巼鎺у埗
  float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD搴旂敤鍙戝竷鐨勬帶鍒舵暟鎹?搴斿綋鐢眊imbal/chassis/shoot璁㈤槄----------------
 */
/**
 * @brief 瀵逛簬鍙屾澘鎯呭喌,閬ユ帶鍣ㄥ拰pc鍦ㄤ簯鍙?閾捐矾鎽樿閫氳繃CAN鍙屽悜鍚屾锛堟敮鎸佸弻閾捐矾鍏滃簳锛?
 *
 */
// cmd鍙戝竷鐨勫簳鐩樻帶鍒舵暟鎹?鐢眂hassis璁㈤槄
typedef struct {
  // 鎺у埗閮ㄥ垎
  float vx;                // 鍓嶈繘鏂瑰悜閫熷害
  float vy;                // 妯Щ鏂瑰悜閫熷害
  float wz;                // 鏃嬭浆閫熷害
  float offset_angle;      // 搴曠洏鍜屽綊涓綅缃殑澶硅
  float near_center_error; // 灏辫繎鍥炰腑璇樊(鑰冭檻缈昏浆浼樺寲鍚?渚汸ID浣跨敤)
  chassis_mode_e chassis_mode;
  int chassis_speed_buff;
  uint8_t vision_is_tracking;      // 瑙嗚鏄惁澶勪簬鐩爣璺熻釜鐘舵€?
  uint8_t image_online;            // 鍥句紶閾捐矾鍦ㄧ嚎鐘舵€?
  uint8_t image_target_locked;     // 鍥句紶鐩爣閿佸畾鐘舵€?
  uint8_t image_auto_fire_request; // 浜戝彴渚ц嚜鍔ㄥ紑鐏姹?
  uint8_t image_should_fire;       // 瑙嗚寤鸿寮€鐏?
  uint16_t image_cmd_seq;          // 鍥句紶鎺у埗搴忓彿
  uint32_t image_ts_ms;            // 图传摘要时间戳(ms)
  uint8_t ui_friction_on;          // UI 用：摩擦轮是否开启
  uint8_t ui_autoaim_enabled;      // UI 用：自瞄模式是否开启
  uint8_t ui_fire_allow;           // UI 用：控制链当前是否允许开火
  uint8_t ui_stuck_active;         // UI 用：卡弹流程是否激活（仅摘要，不含预测量）
  uint8_t ui_loader_mode;          // UI 用：当前发射模式摘要(loader_mode_e)
  uint16_t ui_refresh_request_seq; // UI 用：刷新请求序号，物理扳机上升沿递增
  // UI閮ㄥ垎
  //  ...

} Chassis_Ctrl_Cmd_s;

// cmd鍙戝竷鐨勪簯鍙版帶鍒舵暟鎹?鐢眊imbal璁㈤槄
typedef struct { // 浜戝彴瑙掑害鎺у埗
  float yaw;
  float pitch;
  float chassis_rotate_wz;

  gimbal_mode_e gimbal_mode;

  // 瑙嗚鐩存帴鎺у埗瀛楁锛堢敱robot_cmd浠巚ision_data濉厖锛?
  uint8_t vision_yaw_direct;   // 鏄惁浣跨敤瑙嗚Yaw鐢垫祦鐩存帴鎺у埗
  float vision_yaw_current;    // 瑙嗚Yaw鐢垫祦鎸囦护 [raw]
  uint8_t vision_pitch_direct; // 鏄惁浣跨敤瑙嗚Pitch鐩爣
  float vision_pitch_ref;      // 瑙嗚Pitch鐩爣瑙掑害 [rad]
} Gimbal_Ctrl_Cmd_s;

// cmd鍙戝竷鐨勫彂灏勬帶鍒舵暟鎹?鐢眘hoot璁㈤槄
typedef struct {
  shoot_mode_e shoot_mode;
  loader_mode_e load_mode;
  friction_mode_e friction_mode;
  Bullet_Speed_e bullet_speed; // 弹速枚举
  uint8_t rest_heat;           // 剩余可用热量,用于开火门控与射频前馈
  float shoot_rate;            // 连续发射的射频, unit per s
} Shoot_Ctrl_Cmd_s;

// cmd鍙戝竷鐨勮瑙夋帶鍒舵暟鎹?鐢眝ision璁㈤槄
typedef struct {
  vision_mode_e vision_mode; // 瑙嗚鎺у埗妯″紡
  uint8_t allow_auto_fire;   // 鍏佽鑷姩灏勫嚮
  float manual_yaw_offset;   // 鎵嬪姩寰皟yaw鍋忕Щ閲?
  float manual_pitch_offset; // 鎵嬪姩寰皟pitch鍋忕Щ閲?
} Vision_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis鍙戝竷鐨勫弽棣堟暟鎹?---------------*/
/**
 * @brief 鐢眂md璁㈤槄,鍏朵粬搴旂敤涔熷彲浠ユ牴鎹渶瑕佽幏鍙?
 *
 */

// 甯歌鎵╁睍鍛戒护妗ユ帴濂戠害鐗堟湰涓庤兘鍔涗綅锛圕hassis -> Gimbal锛?
#define REGULAR_BRIDGE_VERSION 1u
#define REGULAR_BRIDGE_CAP_0303 (1u << 0)
#define REGULAR_BRIDGE_CAP_0305 (1u << 1)
#define REGULAR_BRIDGE_CAP_0307 (1u << 2)
#define REGULAR_BRIDGE_CAP_0308 (1u << 3)
#define REGULAR_BRIDGE_CAP_MASK                                              \
  (REGULAR_BRIDGE_CAP_0303 | REGULAR_BRIDGE_CAP_0305 |                      \
   REGULAR_BRIDGE_CAP_0307 | REGULAR_BRIDGE_CAP_0308)
#define REGULAR_BRIDGE_VALID_0303 REGULAR_BRIDGE_CAP_0303
#define REGULAR_BRIDGE_VALID_0305 REGULAR_BRIDGE_CAP_0305
#define REGULAR_BRIDGE_VALID_0307 REGULAR_BRIDGE_CAP_0307
#define REGULAR_BRIDGE_VALID_0308 REGULAR_BRIDGE_CAP_0308

typedef struct {
#if defined(CHASSIS_BOARD) ||                                                  \
    defined(GIMBAL_BOARD) // 闈炲崟鏉跨殑鏃跺€欏簳鐩樿繕灏唅mu鏁版嵁鍥炰紶(鑻ユ湁蹇呰)
                          // attitude_t chassis_imu_data;
#endif
  // 鍚庣画澧炲姞搴曠洏鐨勭湡瀹為€熷害
  // float real_vx;
  // float real_vy;
  // float real_wz;

  uint8_t referee_online;      // 瑁佸垽绯荤粺鍦ㄧ嚎鏍囧織(1:鍦ㄧ嚎,0:绂荤嚎)
  uint16_t current_hp;         // 褰撳墠琛€閲?
  uint16_t buffer_energy;      // 瑁佸垽绯荤粺鍔熺巼缂撳啿鑳介噺
  uint8_t rest_heat;           // 鍓╀綑鏋彛鐑噺
  Bullet_Speed_e bullet_speed; // 寮归€熼檺鍒?
  Enemy_Color_e enemy_color;   // 0 for blue, 1 for red
  // 甯歌閾捐矾鎽樿锛堝簳鐩?>浜戝彴锛?
  uint8_t regular_online;        // 甯歌閾捐矾鍦ㄧ嚎鐘舵€侊紙瑁佸垽閾捐矾锛?
  uint8_t robot_id;              // 瑁佸垽绯荤粺鏈哄櫒浜篒D
  uint16_t chassis_power_limit;  // 瑁佸垽绯荤粺搴曠洏鍔熺巼涓婇檺
  uint16_t barrel_heat;          // 褰撳墠鏋彛鐑噺
  uint16_t barrel_heat_limit;    // 鏋彛鐑噺涓婇檺
  uint16_t barrel_cooling_value; // 鏋彛鍐峰嵈鍊?heat/s)
  float bullet_speed_limit;      // 寮归€熶笂闄愶紙褰撳墠浣跨敤瀹炴祴寮归€熻繎浼硷級
  uint32_t referee_ts_ms;        // 瑁佸垽鎽樿鏃堕棿鎴?ms)
  // 甯歌鎵╁睍鍛戒护妗ユ帴鎽樿锛堝簳鐩?>浜戝彴锛?
  uint8_t regular_bridge_version;    // 妗ユ帴鍗忚鐗堟湰
  uint8_t regular_bridge_capability; // 鏀寔鑳藉姏浣?
  uint8_t regular_cmd_valid_mask;    // 鏈懆鏈熸洿鏂板懡浠や綅鍥?
  uint8_t regular_cmd_seq;           // 妗ユ帴搴忓彿锛堟湁鏂板懡浠ゆ椂閫掑锛?
  float map_target_x;                // 0x0303 鐩爣鐐箈
  float map_target_y;                // 0x0303 鐩爣鐐箉
  uint8_t map_target_robot_id;       // 0x0303 鐩爣鏈哄櫒浜篒D
  uint8_t map_cmd_keyboard;          // 0x0303 閿紶鍛戒护瀛?
  uint16_t map_hero_x;               // 0x0305 鑻遍泟x
  uint16_t map_hero_y;               // 0x0305 鑻遍泟y
  uint16_t map_sentry_x;             // 0x0305 鍝ㄥ叺x
  uint16_t map_sentry_y;             // 0x0305 鍝ㄥ叺y
  uint8_t map_path_intention;        // 0x0307 璺緞鎰忓浘
  uint8_t map_custom_head0;          // 0x0308 鍓?瀛楄妭鎽樿
  uint8_t map_custom_head1;
  uint8_t map_custom_head2;
  uint8_t map_custom_head3;

} Chassis_Upload_Data_s;

// 鍙屾澘CAN閫氫俊鍗曞寘鏈€澶ф湁鏁堣礋杞戒负60瀛楄妭锛岄槻姝㈢粨鏋勪綋鑶ㄨ儉瀵艰嚧瓒婄晫
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(Chassis_Ctrl_Cmd_s) <= 60,
               "Chassis_Ctrl_Cmd_s too large for CAN comm");
_Static_assert(sizeof(Chassis_Upload_Data_s) <= 60,
               "Chassis_Upload_Data_s too large for CAN comm");
#else
typedef char chassis_ctrl_cmd_size_check[(sizeof(Chassis_Ctrl_Cmd_s) <= 60) ? 1
                                                                             : -1];
typedef char chassis_upload_data_size_check[(sizeof(Chassis_Upload_Data_s) <= 60)
                                                ? 1
                                                : -1];
#endif

typedef struct {
  attitude_t gimbal_imu_data;
  uint16_t yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct {
  uint8_t loader_jam_state;          // 鎷ㄧ洏鍗″脊鐘舵€佹満
  uint8_t loader_jam_active;         // 鎷ㄧ洏鏄惁澶勪簬鍗″脊鎭㈠娴佺▼
  uint16_t loader_jam_recovery_count; // 绱鍗″脊鎭㈠娆℃暟
  int16_t loader_real_current;       // 鎷ㄧ洏褰撳墠鍙嶉鐢垫祦锛堢數璋冨師濮嬮噺绾诧級
  float loader_speed_aps;            // 鎷ㄧ洏褰撳墠瑙掗€熷害 [deg/s]
  float loader_total_angle;          // 鎷ㄧ洏褰撳墠鎬昏搴?[deg]
} Shoot_Upload_Data_s;

// vision鍙戝竷鐨勮瑙夊鐞嗘暟鎹?鐢眂md璁㈤槄鐢ㄤ簬铻嶅悎鎺у埗
typedef struct {
  uint8_t vision_valid;  // 瑙嗚鏁版嵁鏈夋晥鏍囧織
  uint8_t target_locked; // 鐩爣閿佸畾鏍囧織
  uint8_t should_fire;   // 寤鸿灏勫嚮鏍囧織

  // 瑙嗚鎺у埗杈撳嚭锛堢敱vision_controller璁＄畻锛?
  uint8_t vision_takeover; // 瑙嗚鎺ョ鏍囧織
  float yaw_current_cmd;   // Yaw鐢垫祦鎸囦护 [raw]锛堣瑙夊弻鐜緭鍑猴級
  float pitch_ref_limited; // Pitch鐩爣瑙掑害 [rad]锛堥檺閫熷悗锛?

  // 鍘熷瑙嗚鐩爣锛堢敤浜庤皟璇?澶囩敤锛?
  float yaw;   // 鍘熷鐩爣yaw瑙掑害 [rad]
  float pitch; // 鍘熷鐩爣pitch瑙掑害 [rad]
} Vision_Upload_Data_s;

/* ----------------绯荤粺杈ㄨ瘑浠诲姟鐩稿叧瀹氫箟----------------*/
// 浜戝彴绯荤粺杈ㄨ瘑杞撮€夋嫨鏋氫妇
typedef enum {
  SYSID_AXIS_YAW = 0,      // 杈ㄨ瘑Yaw杞?
  SYSID_AXIS_PITCH = 1,    // 杈ㄨ瘑Pitch杞?
  SYS_ID_DISABLED_AXIS = 2 // 鏈€夋嫨浠讳綍杞?
} SysID_TargetAxis_e;

// 浜戝彴绯荤粺杈ㄨ瘑鎺у埗鎸囦护锛坓imbal浠诲姟鍙戝竷锛岀郴缁熻鲸璇嗕换鍔¤闃咃級
typedef struct {
  uint8_t enable;  // 浣胯兘鏍囧織锛?-鍚姩杈ㄨ瘑锛?-鍋滄杈ㄨ瘑
  uint8_t axis;    // 鐩爣杞达細0-Yaw 1-Pitch
  float yaw_ref;   // Yaw杞翠綅缃弬鑰冨€硷紙鐢ㄤ簬淇濇寔闈炶鲸璇嗚酱浣嶇疆锛?
  float pitch_ref; // Pitch杞翠綅缃弬鑰冨€硷紙鐢ㄤ簬淇濇寔闈炶鲸璇嗚酱浣嶇疆锛?
} SysID_Ctrl_Cmd_s;

// 绯荤粺杈ㄨ瘑鍙嶉鏁版嵁锛堢郴缁熻鲸璇嗕换鍔″彂甯冿紝浜戝彴浠诲姟璁㈤槄锛?
typedef struct {
  float step_input;      // 鏂规尝杈撳叆淇″彿锛堢數娴佹寚浠わ級
  float motor_output;    // 鐢垫満杈撳嚭鍙嶉锛堥檧铻轰华瑙掗€熷害锛?
  float time_elapsed;    // 宸茶繍琛屾椂闂?[s]
  uint8_t is_finished;   // 杈ㄨ瘑瀹屾垚鏍囧織
  uint8_t step_state;    // 褰撳墠闃惰穬鐘舵€?
  uint32_t call_counter; // 浠诲姟璋冪敤娆℃暟
  float actual_dt;       // 瀹為檯娴嬮噺鐨刣t [s]
  float task_freq;       // 瀹為檯浠诲姟棰戠巼 [Hz]
} SysID_Feedback_s;

#pragma pack() // 寮€鍚瓧鑺傚榻?缁撴潫鍓嶉潰鐨?pragma pack(1)

#endif // !ROBOT_DEF_H




