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
#include "master_process.h"
#include "math.h"
#include "stdint.h"

/* 寮€鍙戞澘绫诲瀷瀹氫箟,鐑у綍鏃舵敞鎰忎笉瑕佸紕閿欏搴斿姛鑳?淇敼瀹氫箟鍚庨渶瑕侀噸鏂扮紪璇?鍙兘瀛樺湪涓€涓畾涔?
 */
// #define ONE_BOARD // 鍗曟澘鎺у埗鏁磋溅
#define CHASSIS_BOARD // 搴曠洏鏉?
// #define GIMBAL_BOARD // 浜戝彴鏉?

#define VISION_USE_VCP // 浣跨敤铏氭嫙涓插彛鍙戦€佽瑙夋暟鎹?
// #define VISION_USE_UART // 浣跨敤涓插彛鍙戦€佽瑙夋暟鎹?

/* 鏈哄櫒浜洪噸瑕佸弬鏁板畾涔?娉ㄦ剰鏍规嵁涓嶅悓鏈哄櫒浜鸿繘琛屼慨鏀?娴偣鏁伴渶瑕佷互.0鎴杅缁撳熬,鏃犵鍙蜂互u缁撳熬
 */
// 浜戝彴鍙傛暟
#define YAW_CHASSIS_ALIGN_ECD                                                  \
  5646 // 浜戝彴鍜屽簳鐩樺榻愭寚鍚戠浉鍚屾柟鍚戞椂鐨勭數鏈虹紪鐮佸櫒鍊?鑻ュ浜戝彴鏈夋満姊版敼鍔ㄩ渶瑕佷慨鏀?
#define YAW_ECD_GREATER_THAN_4096                                              \
  1 // ALIGN_ECD鍊兼槸鍚﹀ぇ浜?096,鏄负1,鍚︿负0;鐢ㄤ簬璁＄畻浜戝彴鍋忚浆瑙掑害
#define PITCH_HORIZON_ECD                                                      \
  0 // 浜戝彴澶勪簬姘村钩浣嶇疆鏃剁紪鐮佸櫒鍊?鑻ュ浜戝彴鏈夋満姊版敼鍔ㄩ渶瑕佷慨鏀?
#define PITCH_MAX_ANGLE                                                        \
  53.5f // 浜戝彴绔栫洿鏂瑰悜鏈€澶ц搴?(娉ㄦ剰鍙嶉濡傛灉鏄檧铻轰华锛屽垯濉啓闄€铻轰华鐨勮搴?
#define PITCH_MIN_ANGLE                                                        \
  -15.5f // 浜戝彴绔栫洿鏂瑰悜鏈€灏忚搴?(娉ㄦ剰鍙嶉濡傛灉鏄檧铻轰华锛屽垯濉啓闄€铻轰华鐨勮搴?
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 60     // 发射一发弹丸时拨盘输出轴转动角度(机械设计值)
#define REDUCTION_RATIO_LOADER 51.0f  // 拨盘减速比,用于输出轴角度和电机轴角度换算
#define NUM_PER_CIRCLE 6              // 拨盘一圈的装载量
// 热量前馈 / UI 热量显示参数(需与 Gimbal 侧保持一致)
#define HEAT_PER_SHOT_D 100.0f
#define FEEDFORWARD_T_TARGET_S 1.0f
#define SHOOT_RATE_MIN 0.5f
#define SHOOT_RATE_MAX 3.0f
#define SHOOT_RATE_SAFE 1.0f
// 机器人底盘参数(单位统一为 m)
#define WHEEL_BASE 0.412f  // 纵向轴距(前进后退方向)，单位 m
#define TRACK_WIDTH 0.412f // 妯悜杞窛(宸﹀彸骞崇Щ鏂瑰悜)锛屽崟浣?m
#define CENTER_GIMBAL_OFFSET_X                                                 \
  0.0f // 浜戝彴鏃嬭浆涓績璺濆簳鐩樺嚑浣曚腑蹇冪殑璺濈,鍓嶅悗鏂瑰悜锛屽崟浣?m
#define CENTER_GIMBAL_OFFSET_Y                                                 \
  0.0f                      // 浜戝彴鏃嬭浆涓績璺濆簳鐩樺嚑浣曚腑蹇冪殑璺濈,宸﹀彸鏂瑰悜锛屽崟浣?m
#define RADIUS_WHEEL 0.077f // 杞瓙鍗婂緞(鍗曚綅m,娉ㄦ剰涓嶆槸鐩村緞)
#define REDUCTION_RATIO_WHEEL                                                  \
  19.0f // 鐢垫満鍑忛€熸瘮,鍥犱负缂栫爜鍣ㄩ噺娴嬬殑鏄浆瀛愮殑閫熷害鑰屼笉鏄緭鍑鸿酱鐨勯€熷害鏁呴渶杩涜杞崲
#define CHASSIS_MASS 25.0f       // 鏈哄櫒浜烘暣澶囪川閲?鍗曚綅:kg
#define GRAVITY_ACCEL 9.81f      // 閲嶅姏鍔犻€熷害,鍗曚綅:m/s^2
#define DIST_CG_FRONT_AXLE 0.28f // 閲嶅績璺濆墠杞磋窛绂?鍗曚綅:m
#define DIST_CG_REAR_AXLE 0.28f  // 閲嶅績璺濆悗杞磋窛绂?鍗曚綅:m
#define CG_HEIGHT 0.132f         // 閲嶅績璺濆簳鐩樹腑蹇冮珮搴?鍗曚綅:m
// 搴曠洏璺熼殢灏辫繎鍥炰腑鍙傛暟
#define CHASSIS_FOLLOW_ALLOW_FLIP 0 // 涓存椂鍏抽棴杞﹀ご缈昏浆锛屼究浜庤皟璇曞簳鐩樿窡闅忓鐜?#define CHASSIS_FOLLOW_FLIP_THRESHOLD 90.0f // 杞﹀ご缈昏浆瑙﹀彂闃堝€?搴?
#define CHASSIS_FOLLOW_MAX_ERR 135.0f       // 鏈€澶у厑璁歌宸?搴?,閬垮厤鎺у埗閲忚繃澶?
// 閿洏鎺у埗鐩稿叧鍙傛暟
//  閿洏鎸変笅鏃剁殑鏈€澶х洰鏍囨寚浠ゅ€?
//  (閬ユ帶鍣ㄦ憞鏉嗘渶澶у€间负660,杩欓噷鍙互鍙傝€冭缃垨璁惧畾鐨勬洿澶?
#define KEYBOARD_CMD_MAX_SPEED_X 2.0f
#define KEYBOARD_CMD_MAX_SPEED_Y 2.0f
// 鍔犻€熷害 (鍗曚綅: 鎸囦护鍊?绉?
// 鍊艰秺澶? 鍝嶅簲瓒婂揩銆傚彲浠ヤ粠 20000 寮€濮嬪皾璇?
#define KEYBOARD_RAMP_ACCEL 3.0f
// 鍏煎鏃у畯鍚嶏細閿洏閫熷害鏈€澶у€硷紙濡傛湁鏃ц皟鐢級
#define CHASSIS_KB_MAX_SPEED_X KEYBOARD_CMD_MAX_SPEED_X
#define CHASSIS_KB_MAX_SPEED_Y KEYBOARD_CMD_MAX_SPEED_Y
// 鍑忛€熷害 (鍗曚綅: 鎸囦护鍊?绉?
// 閫氬父鍙互璁剧疆寰楁瘮ACCEL澶? 瀹炵幇鏇村揩鐨勫埞杞?
#define KEYBOARD_RAMP_DECEL 4.0f
// 鍙嶅悜鍒跺姩鍑忛€熷害 (鍗曚綅: 鎸囦护鍊?绉?
// 鍙互璁剧疆寰楅潪甯稿ぇ, 瀹炵幇鍑屽帀鐨勮浆鍚戝拰鍒跺姩
#define KEYBOARD_RAMP_BRAKE_DECEL 6.0f
/**
 * @brief M3508鐢垫満鎵煩鍒癈AN鎸囦护鍊肩殑杞崲绯绘暟,1N路m瀵瑰簲2730.67鐨凜AN鎸囦护鍊?
 * @note 鏍规嵁瀹樻柟鏁版嵁锛氶瀹氭壄鐭?N路m @ 10A鐢垫祦, C620鐢佃皟 -20A~20A 瀵瑰簲
 * -16384~16384                                璁＄畻鍏紡: (10A / 20A * 16384) /
 * 3N路m = 8192 / 3 鈮?2730.67
 */
#define M3508_TORQUE_TO_CURRENT_CMD_COEFF 2730.67f

/* ----------------鍔涙帶绛栫暐鐩稿叧鐗╃悊鍙傛暟---------------- */
/**
 * @brief M3508鐢垫満杞煩甯告暟 (N路m/A)
 * @note 鏍规嵁瀹樻柟鍙傛暟琛細
 *       - 杞煩甯告暟锛?.3 N路m/A锛堣緭鍑鸿酱锛屽弬鏁拌〃鐩存帴鏍囨敞锛?
 *       - 棰濆畾杞煩锛? N路m @ 10A 鈫?3/10 = 0.3 鉁?楠岃瘉涓€鑷?
 *       - 杩欐槸杈撳嚭杞寸殑杞煩甯告暟锛圡3508 P19 涓€浣撳寲璁捐锛?
 *       - 鍔涙帶鍏紡锛欼 = F 脳 r / Kt锛堜笉闇€瑕佸啀闄ゅ噺閫熸瘮锛?
 */
#define M3508_TORQUE_CONSTANT 0.3f

/**
 * @brief CAN鎸囦护鍊煎埌鐢垫祦鐨勮浆鎹㈢郴鏁?(A per CMD value)
 * @note C620鐢佃皟锛?20A~20A 瀵瑰簲 -16384~16384锛岀郴鏁?= 20/16384 鈮?0.00122
 */
#define M3508_CMD_TO_CURRENT_COEFF (20.0f / 16384.0f)

/**
 * @brief 鍔涙帶绛栫暐鎽╂摝琛ュ伩鍙傛暟
 */
// 闈欐懇鎿﹁ˉ鍋跨數娴佸€?(A)锛岄渶瑕佹牴鎹疄闄呮満鍣ㄤ汉鏍囧畾
#define FRICTION_STATIC_CURRENT 0.75f
// 鍔ㄦ懇鎿﹁ˉ鍋跨數娴佸€?(A)锛岄渶瑕佹牴鎹疄闄呮満鍣ㄤ汉鏍囧畾
#define FRICTION_DYNAMIC_CURRENT 0.5f

// 鏈€澶ф帶鍒跺姏 (N)
#define MAX_CONTROL_FORCE 300.0f
// 鏈€澶ф帶鍒舵壄鐭?(N路m)
#define MAX_CONTROL_TORQUE 100.0f
// 鍗曡疆鏈€澶х數娴?(A)
#define MAX_WHEEL_CURRENT 20.0f

/* ----------------搴曠洏杩愯鏃堕厤缃粨鏋勪綋---------------- */
/**
 * @brief 搴曠洏閬ユ帶鍣ㄦ帶鍒堕厤缃?
 */
typedef struct {
  float max_linear_speed;  // m/s - 鏈€澶х嚎閫熷害
  float max_angular_speed; // rad/s - 鏈€澶ц閫熷害
} Chassis_RC_Config_t;

/**
 * @brief 搴曠洏鍔涙帶绛栫暐閰嶇疆
 */
typedef struct {
  float torque_feedforward_coeff;   // N路m/(rad/s) - 鎵煩鍓嶉绯绘暟
  float friction_threshold_omega;   // rad/s - 鎽╂摝琛ュ伩閫熷害闃堝€?
  float wheel_speed_feedback_coeff; // A路s/rad - 杞€熷弽棣堢郴鏁?
  float omega_error_lpf_alpha;      // 瑙掗€熷害璇樊婊ゆ尝绯绘暟
  float omega_threshold;            // rad/s - 杩囬浂淇濇姢瑙掗€熷害闃堝€?
} Chassis_Force_Control_Config_t;

/**
 * @brief 搴曠洏杩愬姩瀛﹂厤缃?
 */
typedef struct {
  float velocity_lpf_alpha; // 閫熷害浼扮畻婊ゆ尝绯绘暟
  float speed_deadband;     // 閫熷害姝诲尯闃堝€?(deg/s)
  float rotate_speed;       // 灏忛檧铻烘ā寮忔棆杞€熷害 (rad/s)
} Chassis_Kinematics_Config_t;

/**
 * @brief 搴曠洏瀹屾暣閰嶇疆闆嗗悎
 */
typedef struct {
  Chassis_RC_Config_t rc;                 // 閬ユ帶鍣ㄩ厤缃?
  Chassis_Force_Control_Config_t force;   // 鍔涙帶閰嶇疆
  Chassis_Kinematics_Config_t kinematics; // 杩愬姩瀛﹂厤缃?
} Chassis_Runtime_Config_t;

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
  GIMBAL_ZERO_FORCE = 0, // 鐢垫祦闆惰緭鍏?
  GIMBAL_FREE_MODE, // 浜戝彴鑷敱杩愬姩妯″紡,鍗充笌搴曠洏鍒嗙(搴曠洏姝ゆ椂搴斾负NO_FOLLOW)鍙嶉鍊间负鐢垫満total_angle;浼间箮鍙互鏀逛负鍏ㄩ儴鐢↖MU鏁版嵁?
  GIMBAL_GYRO_MODE, // 浜戝彴闄€铻轰华鍙嶉妯″紡,鍙嶉鍊间负闄€铻轰华pitch,total_yaw_angle,搴曠洏鍙互涓哄皬闄€铻哄拰璺熼殢妯″紡
  GIMBAL_SYS_ID_CHIRP, // 浜戝彴姝ｅ鸡鎵杈ㄨ瘑妯″紡,鐢ㄤ簬绯荤粺杈ㄨ瘑鍜孭ID鏁村畾
} gimbal_mode_e;

// 鍙戝皠妯″紡璁剧疆
typedef enum {
  SHOOT_OFF = 0,
  SHOOT_ON,
} shoot_mode_e;
typedef enum {
  FRICTION_OFF = 0, // 鎽╂摝杞叧闂?
  FRICTION_ON,      // 鎽╂摝杞紑鍚?
} friction_mode_e;

typedef enum {
  LOAD_STOP = 0,  // 鍋滄鍙戝皠
  LOAD_REVERSE,   // 鍙嶈浆
  LOAD_1_BULLET,  // 鍗曞彂
  LOAD_3_BULLET,  // 涓夊彂
  LOAD_BURSTFIRE, // 杩炲彂
} loader_mode_e;

// 鍔熺巼闄愬埗,浠庤鍒ょ郴缁熻幏鍙?鏄惁鏈夊繀瑕佷繚鐣?
typedef struct { // 鍔熺巼鎺у埗
  float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD搴旂敤鍙戝竷鐨勬帶鍒舵暟鎹?搴斿綋鐢眊imbal/chassis/shoot璁㈤槄----------------
 */
/**
 * @brief
 * 瀵逛簬鍙屾澘鎯呭喌,閬ユ帶鍣ㄥ拰pc鍦ㄤ簯鍙?閾捐矾鎽樿閫氳繃CAN鍙屽悜鍚屾锛堟敮鎸佸弻閾捐矾鍏滃簳锛?
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
} Gimbal_Ctrl_Cmd_s;

// cmd鍙戝竷鐨勫彂灏勬帶鍒舵暟鎹?鐢眘hoot璁㈤槄
typedef struct {
  shoot_mode_e shoot_mode;
  loader_mode_e load_mode;
  friction_mode_e friction_mode;
  Bullet_Speed_e bullet_speed; // 寮归€熸灇涓?
  uint8_t rest_heat;
  float shoot_rate; // 杩炵画鍙戝皠鐨勫皠棰?unit per s,鍙?绉?
} Shoot_Ctrl_Cmd_s;

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
#define REGULAR_BRIDGE_CAP_MASK                                                \
  (REGULAR_BRIDGE_CAP_0303 | REGULAR_BRIDGE_CAP_0305 |                         \
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
typedef char
    chassis_ctrl_cmd_size_check[(sizeof(Chassis_Ctrl_Cmd_s) <= 60) ? 1 : -1];
typedef char
    chassis_upload_data_size_check[(sizeof(Chassis_Upload_Data_s) <= 60) ? 1
                                                                         : -1];
#endif

typedef struct {
  attitude_t gimbal_imu_data;
  uint16_t yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct {
  // code to go here
  // ...
} Shoot_Upload_Data_s;

/* ----------------绯荤粺杈ㄨ瘑浠诲姟鐩稿叧瀹氫箟----------------*/
// 搴曠洏绯荤粺杈ㄨ瘑鎺у埗鎸囦护锛坈md浠诲姟鍙戝竷锛岀郴缁熻鲸璇嗕换鍔¤闃咃級
typedef struct {
  uint8_t enable;       // 浣胯兘鏍囧織锛?-鍚姩杈ㄨ瘑锛?-鍋滄杈ㄨ瘑
  uint8_t target_motor; // 鐩爣鐢垫満锛?-lf, 1-rf, 2-lb, 3-rb
} Chassis_SysID_Ctrl_Cmd_s;

// 搴曠洏绯荤粺杈ㄨ瘑鍙嶉鏁版嵁锛堢郴缁熻鲸璇嗕换鍔″彂甯冿紝cmd浠诲姟璁㈤槄锛?
typedef struct {
  float step_input;      // 鏂规尝杈撳叆淇″彿锛堢數娴丆AN鎸囦护鍊硷級
  float motor_output;    // 鐢垫満杈撳嚭鍙嶉锛堣疆閫?rad/s锛?
  float time_elapsed;    // 宸茶繍琛屾椂闂?[s]
  uint8_t is_finished;   // 杈ㄨ瘑瀹屾垚鏍囧織
  uint8_t step_state;    // 褰撳墠闃惰穬鐘舵€侊紙0-姝ｅ悜锛?-鍙嶅悜锛?
  uint32_t call_counter; // 浠诲姟璋冪敤娆℃暟
  float actual_dt;       // 瀹為檯娴嬮噺鐨刣t [s]
  float task_freq;       // 瀹為檯浠诲姟棰戠巼 [Hz]
} Chassis_SysID_Feedback_s;

#pragma pack() // 寮€鍚瓧鑺傚榻?缁撴潫鍓嶉潰鐨?pragma pack(1)

#endif // !ROBOT_DEF_H



