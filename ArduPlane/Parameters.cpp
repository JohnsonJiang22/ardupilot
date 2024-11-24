#include "Plane.h"

/*
 *  ArduPlane parameter definitions
 *
 */
// GSCALAR: 声明一个通用全局变量
// ASCALAR: 声明一个专用变量
// GOBJECT: 创建、初始化一个实例
// GOBJECTN: 可用于在运行时动态地创建和管理对象实例，从而支持更灵活的飞行控制策略
#define GSCALAR(v, name, def) { plane.g.v.vtype, name, Parameters::k_param_ ## v, &plane.g.v, {def_value : def} }
#define ASCALAR(v, name, def) { plane.aparm.v.vtype, name, Parameters::k_param_ ## v, (const void *)&plane.aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &plane.g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&plane.v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, (const void *)&plane.v, {group_info : class::var_info} }

const AP_Param::Info Plane::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // @Param: SYSID_THISMAV
    // @DisplayName: MAVLink system ID of this vehicle
    // @Description: Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_this_mav,         "SYSID_THISMAV",  MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: Ground station MAVLink system ID
    // @Description: The identifier of the ground station in the MAVLink protocol. Don't change this unless you also modify the ground station to match.
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_my_gcs,           "SYSID_MYGCS",    255),

    // @Group: SERIAL
    // @Path: ../libraries/AP_SerialManager/AP_SerialManager.cpp
    GOBJECT(serial_manager, "SERIAL",   AP_SerialManager),

    // @Param: AUTOTUNE_LEVEL autotune是ArduPilot飞行控制软件中一种重要的自动调参功能，它允许飞控系统在学习飞行姿态的过程中自动调整PID（比例-积分-微分）参数，以优化飞行性能
    // @DisplayName: Autotune level
    // 俯仰和滚转PID增益的激进程度。较低的值会产生“更柔和”的调校效果。对于大多数飞机，推荐使用6级。如果值为0，则意味着保持控制器当前的RMAX和TCONST值不变，仅调整PID值
    // @Description: Level of aggressiveness of pitch and roll PID gains. Lower values result in a 'softer' tune. Level 6 recommended for most planes. A value of 0 means to keep the current values of RMAX and TCONST for the controllers, tuning only the PID values
    // @Range: 0 10
    // @Increment: 1
    // @User: Standard
    ASCALAR(autotune_level, "AUTOTUNE_LEVEL",  6),

    // @Param: TELEM_DELAY 无线电遥测数据的延迟时间
    // @DisplayName: Telemetry startup delay 
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Standard
    // @Units: s
    // @Range: 0 30
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: GCS_PID_MASK
    // @DisplayName: GCS PID tuning mask
    // 用于发送MAVLink PID_TUNING消息的PID位掩码,通过设置一个位掩码来指定哪些PID参数的信息应该通过MAVLink协议以PID_TUNING消息的形式发送
    // @Description: bitmask of PIDs to send MAVLink PID_TUNING messages for
    // @User: Advanced
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw,3:Steering,4:Landing
    GSCALAR(gcs_pid_mask,           "GCS_PID_MASK",     0),

    // @Param: KFF_RDDRMIX
    // @DisplayName: Rudder Mix
    // 在副翼动作时添加的舵量。如果飞机在初步滚转时机头偏离滚转方向偏航，则增加此值。这可以减少不利的偏航。
    // @Description: Amount of rudder to add during aileron movement. Increase if nose initially yaws away from roll. Reduces adverse yaw.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(kff_rudder_mix,         "KFF_RDDRMIX",    RUDDER_MIX),

    // @Param: KFF_THR2PTCH
    // @DisplayName: Throttle to Pitch Mix
    // 根据油门比例增加俯仰角。当油门达到100%时，将在俯仰目标角度上增加这个数值的度数
    // @Description: Pitch up to add in proportion to throttle. 100% throttle will add this number of degrees to the pitch target.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    GSCALAR(kff_throttle_to_pitch,  "KFF_THR2PTCH",   0),

    // @Param: STAB_PITCH_DOWN 用于控制飞行器在稳定模式下俯仰角向下的程度或速度
    // @DisplayName: Low throttle pitch down trim 
    // @Description: Degrees of down pitch added when throttle is below TRIM_THROTTLE in FBWA and AUTOTUNE modes. Scales linearly so full value is added when THR_MIN is reached. Helps to keep airspeed higher in glides or landing approaches and prevents accidental stalls. 2 degrees recommended for most planes.
    // @Range: 0 15
    // @Increment: 0.1
    // @Units: deg
    // @User: Advanced
    GSCALAR(stab_pitch_down, "STAB_PITCH_DOWN",   2.0f),

    // @Param: GLIDE_SLOPE_MIN
    // @DisplayName: Glide slope minimum
    // 主要与飞行器的下滑道（glide slope）控制相关。下滑道是指飞行器在接近着陆时，沿着一个预定的倾斜路径下降的过程。这个参数设置了一个下滑道倾斜度的最小值，用于确保飞行器在着陆过程中能够保持一个安全且稳定的下降角度
    // @Description: This controls the minimum altitude change for a waypoint before a glide slope will be used instead of an immediate altitude change. The default value is 15 meters, which helps to smooth out waypoint missions where small altitude changes happen near waypoints. If you don't want glide slopes to be used in missions then you can set this to zero, which will disable glide slope calculations. Otherwise you can set it to a minimum number of meters of altitude error to the destination waypoint before a glide slope will be used to change altitude.
    // @Range: 0 1000
    // @Increment: 1
    // @Units: m
    // @User: Advanced
    GSCALAR(glide_slope_min, "GLIDE_SLOPE_MIN", 15),

    // @Param: GLIDE_SLOPE_THR 用于设定下滑斜率的阈值
    // @DisplayName: Glide slope threshold
    // @Description: This controls the height above the glide slope the plane may be before rebuilding a glide slope. This is useful for smoothing out an autotakeoff
    // @Range: 0 100
    // @Increment: 1
    // @Units: m
    // @User: Advanced
    GSCALAR(glide_slope_threshold, "GLIDE_SLOPE_THR", 5.0),

    // @Param: STICK_MIXING
    // @DisplayName: Stick Mixing
    // 启用该功能后，它会在自动模式下将用户的摇杆输入添加到控制舵面上，使用户能够在不改变模式的情况下获得一定程度的飞行控制。有两种可用的摇杆混合方式。
    // 如果将STICK_MIXING设置为1，则会使用“线控飞行”（Fly By Wire）混合，该方式与FBWA（Fly By Wire A）模式以相同的方式控制滚转和俯仰。
    // 如果您通常在FBWA或FBWB（Fly By Wire B）模式下驾驶ArduPlane，这是最安全的选择。如果将STICK_MIXING设置为2，则会启用直接混合模式，
    // 这是STABILIZE（稳定）模式所使用的。这样，在自动模式下可以进行更加极端的机动动作。如果将STICK_MIXING设置为3，
    // 则仅在四旋翼模式（如进行自动垂直起降VTOL起飞或着陆时）下对偏航生效
    // @Description: When enabled, this adds user stick input to the control surfaces in auto modes, allowing the user to have some degree of flight control without changing modes.  There are two types of stick mixing available. If you set STICK_MIXING to 1 then it will use "fly by wire" mixing, which controls the roll and pitch in the same way that the FBWA mode does. This is the safest option if you usually fly ArduPlane in FBWA or FBWB mode. If you set STICK_MIXING to 2 then it will enable direct mixing mode, which is what the STABILIZE mode uses. That will allow for much more extreme maneuvers while in AUTO mode. If you set STICK_MIXING to 3 then it will apply to the yaw while in quadplane modes only, such as while doing an automatic VTOL takeoff or landing.
    // @Values: 0:Disabled,1:FBWMixing,2:DirectMixing,3:VTOL Yaw only
    // @User: Advanced
    GSCALAR(stick_mixing,           "STICK_MIXING",   uint8_t(StickMixing::FBW)),

    // @Param: TKOFF_THR_MINSPD
    // @DisplayName: Takeoff throttle min speed
    // 在自动起飞过程中，用于速度检查的最小地面GPS速度（以米/秒为单位），该检查用于在解除油门抑制时启用。这可以用于弹射起飞，
    // 即你希望仅在飞机离开弹射装置后发动机才启动的情况。然而，由于GPS测量存在的误差，对于弹射起飞，更推荐使用TKOFF_THR_MINACC（最小加速度）和TKOFF_THR_DELAY（延迟时间）参数。
    // 对于使用拉式螺旋桨的手掷起飞，强烈建议将此参数设置为不小于4米/秒的值，以提供额外的保护，防止发动机过早启动。请注意，GPS速度会比实际速度滞后约0.5秒。
    // 地面速度检查的延迟由TKOFF_THR_DELAY参数决定。
    // @Description: Minimum GPS ground speed in m/s used by the speed check that un-suppresses throttle in auto-takeoff. This can be be used for catapult launches where you want the motor to engage only after the plane leaves the catapult, but it is preferable to use the TKOFF_THR_MINACC and TKOFF_THR_DELAY parameters for catapult launches due to the errors associated with GPS measurements. For hand launches with a pusher prop it is strongly advised that this parameter be set to a value no less than 4 m/s to provide additional protection against premature motor start. Note that the GPS velocity will lag the real velocity by about 0.5 seconds. The ground speed check is delayed by the TKOFF_THR_DELAY parameter.
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(takeoff_throttle_min_speed,     "TKOFF_THR_MINSPD",  0),

    // @Param: TKOFF_THR_MINACC
    // @DisplayName: Takeoff throttle min acceleration
    // 在自动起飞中进行地面速度检查之前所需的最小前向加速度（以米/秒²为单位）。这一设置旨在用于手掷起飞。将此值设置为0会禁用加速度测试，
    // 这意味着地面速度检查将始终被激活，这可能会导致GPS速度突变从而启动发动机。对于手掷起飞和弹力绳发射起飞，应将其设置为约15。另外，
    // 请参见TKOFF_ACCEL_CNT参数，以控制完整的‘摇动以激活’功能
    // @Description: Minimum forward acceleration in m/s/s before arming the ground speed check in auto-takeoff. This is meant to be used for hand launches. Setting this value to 0 disables the acceleration test which means the ground speed check will always be armed which could allow GPS velocity jumps to start the engine. For hand launches and bungee launches this should be set to around 15. Also see TKOFF_ACCEL_CNT paramter for control of full "shake to arm".
    // @Units: m/s/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(takeoff_throttle_min_accel,     "TKOFF_THR_MINACC",  0),

    // @Param: TKOFF_THR_DELAY
    // @DisplayName: Takeoff throttle delay
    // 此参数设置地面速度检查在由TKOFF_THR_MINACC控制的向前加速度检查通过后延迟的时间（以十分之一秒为单位）。对于使用前推式螺旋桨的手掷起飞，
    // 将此参数设置为不小于2（0.2秒）的值至关重要，以确保飞机在电机启动前已安全脱离投掷者的手臂。对于使用蹦床或弹性绳发射的起飞方式，
    // 可以使用更大的值（如30）来确保在电机启动前，蹦床或弹性绳有足够的时间从飞机上释放
    // @Description: This parameter sets the time delay (in 1/10ths of a second) that the ground speed check is delayed after the forward acceleration check controlled by TKOFF_THR_MINACC has passed. For hand launches with pusher propellers it is essential that this is set to a value of no less than 2 (0.2 seconds) to ensure that the aircraft is safely clear of the throwers arm before the motor can start. For bungee launches a larger value can be used (such as 30) to give time for the bungee to release from the aircraft before the motor is started.
    // @Units: ds
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    GSCALAR(takeoff_throttle_delay,     "TKOFF_THR_DELAY",  2),

    // @Param: TKOFF_TDRAG_ELEV
    // @DisplayName: Takeoff tail dragger elevator
    // 此参数用于设置在起飞初始阶段应用升降舵的量。它用于在起飞初始阶段将尾轮式飞机的尾轮保持在地面上，以提供最大的转向能力。
    // 此选项应与TKOFF_TDRAG_SPD1选项、GROUND_STEER_ALT选项以及地面转向控制器的调整结合使用。如果此参数设置为零，则表示跳过起飞的初始“尾轮保持”阶段。
    // 对于手投或弹射起飞，应将其设置为零。对于尾轮式飞机，通常应将其设置为100，意味着在起飞初始阶段升降舵完全向上。对于大多数前三点式起落架飞机，
    // 零值即可很好地工作，但对于某些前三点式飞机，使用较小的负值（例如-20至-30左右）会使升降舵向下，从而在初始加速阶段将机头轮牢固地保持在地面上。
    // 仅在发现机头轮在起飞时抓地力不佳时才使用负值。前三点式起落架上过多的向下升降舵可能会导致飞机围绕机头轮转动时转向不稳定。每次增加10%的向下升降舵
    // @Description: This parameter sets the amount of elevator to apply during the initial stage of a takeoff. It is used to hold the tail wheel of a taildragger on the ground during the initial takeoff stage to give maximum steering. This option should be combined with the TKOFF_TDRAG_SPD1 option and the GROUND_STEER_ALT option along with tuning of the ground steering controller. A value of zero means to bypass the initial "tail hold" stage of takeoff. Set to zero for hand and catapult launch. For tail-draggers you should normally set this to 100, meaning full up elevator during the initial stage of takeoff. For most tricycle undercarriage aircraft a value of zero will work well, but for some tricycle aircraft a small negative value (say around -20 to -30) will apply down elevator which will hold the nose wheel firmly on the ground during initial acceleration. Only use a negative value if you find that the nosewheel doesn't grip well during takeoff. Too much down elevator on a tricycle undercarriage may cause instability in steering as the plane pivots around the nosewheel. Add down elevator 10 percent at a time.
    // @Units: %
    // @Range: -100 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(takeoff_tdrag_elevator,     "TKOFF_TDRAG_ELEV",  0),

    // @Param: TKOFF_TDRAG_SPD1
    // @DisplayName: Takeoff tail dragger speed1
    // 此参数设置的是飞机停止保持尾部向下并过渡到地面方向舵控制的空速。当达到TKOFF_TDRAG_SPD1（起飞拖曳速度1）时，飞机的俯仰角将保持水平状态，
    // 直到达到TKOFF_ROTATE_SPD（起飞旋转速度），此时将使用任务中指定的起飞俯仰角来“旋转”俯仰角以实现起飞爬升。将TKOFF_TDRAG_SPD1设置为零可直接进入旋转阶段。
    // 对于手掷发射和弹射发射，应将其设置为零。对于前三点式起落架，除非您使用上述方法轻轻保持前轮向下，否则也应将其设置为零。对于尾轮式飞机，
    // 应将其设置在失速速度以下
    // @Description: This parameter sets the airspeed at which to stop holding the tail down and transition to rudder control of steering on the ground. When TKOFF_TDRAG_SPD1 is reached the pitch of the aircraft will be held level until TKOFF_ROTATE_SPD is reached, at which point the takeoff pitch specified in the mission will be used to "rotate" the pitch for takeoff climb. Set TKOFF_TDRAG_SPD1 to zero to go straight to rotation. This should be set to zero for hand launch and catapult launch. It should also be set to zero for tricycle undercarriages unless you are using the method above to genetly hold the nose wheel down. For tail dragger aircraft it should be set just below the stall speed.
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(takeoff_tdrag_speed1,     "TKOFF_TDRAG_SPD1",  0),

    // @Param: TKOFF_ROTATE_SPD
    // @DisplayName: Takeoff rotate speed
    // 此参数设定飞机“拉起”（即爬升俯仰）时的空速，并确定任务中指定的爬升俯仰角。如果TKOFF_ROTATE_SPD（起飞拉起速度）设置为零，
    // 那么飞机一开始起飞就会使用设定的爬升俯仰角。对于手掷发射和弹射发射，应将TKOFF_ROTATE_SPD设置为零。对于所有地面发射，
    // TKOFF_ROTATE_SPD应设置在失速速度之上，通常高出10%到30%左右
    // @Description: This parameter sets the airspeed at which the aircraft will "rotate", setting climb pitch specified in the mission. If TKOFF_ROTATE_SPD is zero then the climb pitch will be used as soon as takeoff is started. For hand launch and catapult launches a TKOFF_ROTATE_SPD of zero should be set. For all ground launches TKOFF_ROTATE_SPD should be set above the stall speed, usually by about 10 to 30 percent
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(takeoff_rotate_speed,     "TKOFF_ROTATE_SPD",  0),

    // @Param: TKOFF_THR_SLEW
    // @DisplayName: Takeoff throttle slew rate
    // 用于设置自动起飞期间油门的变化率（即斜率）。当此参数为零时，起飞过程中将使用THR_SLEWRATE参数。对于滚动起飞而言，
    // 设置一个较低的起飞变化率是一个不错的选择，因为这可以提供较慢的加速度，从而改善地面转向控制。该值表示每秒油门变化的百分比，
    // 因此，值为20意味着起飞时油门将在5秒内逐渐增加。不建议使用低于20的值，因为可能导致飞机以过小的油门尝试爬升。值为-1表示起飞时对变化率没有限制
    // @Description: This parameter sets the slew rate for the throttle during auto takeoff. When this is zero the THR_SLEWRATE parameter is used during takeoff. For rolling takeoffs it can be a good idea to set a lower slewrate for takeoff to give a slower acceleration which can improve ground steering control. The value is a percentage throttle change per second, so a value of 20 means to advance the throttle over 5 seconds on takeoff. Values below 20 are not recommended as they may cause the plane to try to climb out with too little throttle. A value of -1 means no limit on slew rate in takeoff.
    // @Units: %/s
    // @Range: -1 127
    // @Increment: 1
    // @User: Standard
    GSCALAR(takeoff_throttle_slewrate, "TKOFF_THR_SLEW",  0),

    // @Param: TKOFF_PLIM_SEC
    // @DisplayName: Takeoff pitch limit reduction
    // 此参数在自动起飞即将到达目标高度前的几秒钟内，略微降低了俯仰角的最小限制。这样做可以减少超调现象，使飞行控制器能够在到达目标高度前的几秒钟就开始进行水平飞行调整。
    // 当此参数设置为零时，任务过程中的俯仰角最小值将一直维持到目标高度并贯穿整个目标高度段，否则在最终阶段，俯仰角最小值将缓慢减小至零。这里指的是俯仰角的最小值（pitch_min），
    // 而不是需求值。飞行控制器仍然会发出指令以增加高度以完成起飞，但有了这个参数，它就不会被强制提升到高于期望的高度
    // @Description: This parameter reduces the pitch minimum limit of an auto-takeoff just a few seconds before it reaches the target altitude. This reduces overshoot by allowing the flight controller to start leveling off a few seconds before reaching the target height. When set to zero, the mission pitch min is enforced all the way to and through the target altitude, otherwise the pitch min slowly reduces to zero in the final segment. This is the pitch_min, not the demand. The flight controller should still be commanding to gain altitude to finish the takeoff but with this param it is not forcing it higher than it wants to be.
    // @Units: s
    // @Range: 0 10
    // @Increment: 0.5
    // @User: Advanced
    GSCALAR(takeoff_pitch_limit_reduction_sec, "TKOFF_PLIM_SEC",  2),

    // @Param: TKOFF_FLAP_PCNT
    // @DisplayName: Takeoff flap percentage
    // @Description: The amount of flaps (as a percentage) to apply in automatic takeoff 自动起飞时应使用的襟翼量（百分比）
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    GSCALAR(takeoff_flap_percent,     "TKOFF_FLAP_PCNT", 0),

    // @Param: LEVEL_ROLL_LIMIT
    // @DisplayName: Level flight roll limit
    // 在期望水平飞行的飞行模式下（如着陆的最后阶段和自动起飞期间），这可以控制最大倾斜角（以度为单位）。为防止起飞或着陆时机翼撞到跑道，
    // 该角度应较小（如5度）。将此设置为零将完全禁用自动起飞和最终着陆进近时的航向保持功能。
    // @Description: This controls the maximum bank angle in degrees during flight modes where level flight is desired, such as in the final stages of landing, and during auto takeoff. This should be a small angle (such as 5 degrees) to prevent a wing hitting the runway during takeoff or landing. Setting this to zero will completely disable heading hold on auto takeoff and final landing approach.
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Standard
    GSCALAR(level_roll_limit,              "LEVEL_ROLL_LIMIT",   5),

    // @Param: USE_REV_THRUST
    // @DisplayName: Bitmask for when to allow negative reverse thrust
    // 这个设置控制何时使用反推力。如果将其设置为零，则永远不会使用反推力。如果设置为非零值，则这些位对应于可能使用反推力的飞行阶段。
    // USE_REV_THRUST最常用的值是2，这意味着仅在自动着陆阶段使用。这样可以在自动模式的着陆阶段启用反推力。另一个常见选择是1，
    // 意味着在所有自动飞行阶段都使用反推力。如果在手动模式下启用，并且THR_MIN（最小推力）小于0，则始终使用反推力。在非自动油门控制模式下，
    // 如果不使用反推力，则该模式下的THR_MIN实际上被设置为0
    // @Description: This controls when to use reverse thrust. If set to zero then reverse thrust is never used. If set to a non-zero value then the bits correspond to flight stages where reverse thrust may be used. The most commonly used value for USE_REV_THRUST is 2, which means AUTO_LAND only. That enables reverse thrust in the landing stage of AUTO mode. Another common choice is 1, which means to use reverse thrust in all auto flight stages. Reverse thrust is always used in MANUAL mode if enabled with THR_MIN < 0. In non-autothrottle controlled modes, if reverse thrust is not used, then THR_MIN is effectively set to 0 for that mode.
    // @Values: 0:MANUAL ONLY,1:AutoAlways,2:AutoLanding
    // @Bitmask: 0:AUTO_ALWAYS,1:AUTO_LAND,2:AUTO_LOITER_TO_ALT,3:AUTO_LOITER_ALL,4:AUTO_WAYPOINTS,5:LOITER,6:RTL,7:CIRCLE,8:CRUISE,9:FBWB,10:GUIDED,11:AUTO_LANDING_PATTERN,12:FBWA,13:ACRO,14:STABILIZE,15:THERMAL
    // @User: Advanced
    GSCALAR(use_reverse_thrust,     "USE_REV_THRUST",  USE_REVERSE_THRUST_AUTO_LAND_APPROACH),

    // @Param: ALT_OFFSET
    // @DisplayName: Altitude offset
    // 在自动飞行中，这会被加到目标高度上。它可以用于为一项任务添加一个全局高度偏移量。
    // @Description: This is added to the target altitude in automatic flight. It can be used to add a global altitude offset to a mission
    // @Units: m
    // @Range: -32767 32767
    // @Increment: 1
    // @User: Advanced
    GSCALAR(alt_offset, "ALT_OFFSET",                 0),

    // @Param: WP_RADIUS
    // @DisplayName: Waypoint Radius
    // 定义了从航点出发的最大距离，一旦越过该距离，即表示航点可能已完成。为避免飞机因偏离航点超过WP_RADIUS（航点半径）而围绕航点飞行，会进行额外的检查，
    // 以确定飞机是否越过了通过该航点并与从上一个航点延伸的飞行路径垂直的“终点线”。如果飞机越过了该终点线，则航点被视为已完成。请注意，
    // 导航控制器可能会根据转弯的急缓程度和飞机的速度，决定在到达航点之前的WP_RADIUS距离之外更晚的时刻进行转弯。将WP_RADIUS设置为远大于飞机通常的转弯半径是安全的，
    // 导航控制器会计算出何时进行转弯。如果将WP_RADIUS设置得太小，飞机可能会在转弯时飞过头
    // @Description: Defines the maximum distance from a waypoint that when crossed indicates the waypoint may be complete. To avoid the aircraft looping around the waypoint in case it misses by more than the WP_RADIUS an additional check is made to see if the aircraft has crossed a "finish line" passing through the waypoint and perpendicular to the flight path from the previous waypoint. If that finish line is crossed then the waypoint is considered complete. Note that the navigation controller may decide to turn later than WP_RADIUS before a waypoint, based on how sharp the turn is and the speed of the aircraft. It is safe to set WP_RADIUS much larger than the usual turn radius of your aircraft and the navigation controller will work out when to turn. If you set WP_RADIUS too small then you will tend to overshoot the turns.
    // @Units: m
    // @Range: 1 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(waypoint_radius,        "WP_RADIUS",      WP_RADIUS_DEFAULT),

    // @Param: WP_MAX_RADIUS
    // @DisplayName: Waypoint Maximum Radius
    // 设置到达路径点被视为完成时的最大距离。这会覆盖通常用于判断路径点完成的“穿过终点线”逻辑。对于正常的AUTO（自动）行为，此参数应设置为零。
    // 仅当飞机必须在给定半径内接近目标且应围绕目标盘旋直至达到该条件时，才建议使用非零值。如果飞机的转弯半径大于设置的最大半径，这可能会导致飞机无限盘旋。
    // @Description: Sets the maximum distance to a waypoint for the waypoint to be considered complete. This overrides the "cross the finish line" logic that is normally used to consider a waypoint complete. For normal AUTO behaviour this parameter should be set to zero. Using a non-zero value is only recommended when it is critical that the aircraft does approach within the given radius, and should loop around until it has done so. This can cause the aircraft to loop forever if its turn radius is greater than the maximum radius set.
    // @Units: m
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(waypoint_max_radius,        "WP_MAX_RADIUS",      0),

    // @Param: WP_LOITER_RAD
    // @DisplayName: Waypoint Loiter Radius
    // 定义了飞机在盘旋时与航点中心保持的距离。如果将此值设置为负数，则默认的盘旋方向将为逆时针方向，而不是顺时针方向
    // @Description: Defines the distance from the waypoint center, the plane will maintain during a loiter. If you set this value to a negative number then the default loiter direction will be counter-clockwise instead of clockwise.
    // @Units: m
    // @Range: -32767 32767
    // @Increment: 1
    // @User: Standard
    ASCALAR(loiter_radius,          "WP_LOITER_RAD",  LOITER_RADIUS_DEFAULT),

    // @Param: RTL_RADIUS
    // @DisplayName: RTL loiter radius
    // 此参数定义了RTL（返回起飞点）模式下盘旋圆圈的半径。如果此值为零，则使用WP_LOITER_RAD参数。如果半径为负数，则盘旋方向为逆时针；如果为正数，则盘旋方向为顺时针
    // @Description: Defines the radius of the loiter circle when in RTL mode. If this is zero then WP_LOITER_RAD is used. If the radius is negative then a counter-clockwise is used. If positive then a clockwise loiter is used.
    // @Units: m
    // @Range: -32767 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_radius,             "RTL_RADIUS",  0),
    
    // @Param: STALL_PREVENTION
    // @DisplayName: Enable stall prevention
    // 在限制滚转的飞行模式下，当空速较低时启用滚转限制。滚转限制基于转弯时的气动载荷因子，并根据ARSPD_FBW_MIN（必须正确设置）进行缩放。
    // 如果没有空速传感器，则使用基于风速估计的合成空速，但合成空速可能不准确
    // @Description: Enables roll limits at low airspeed in roll limiting flight modes. Roll limits based on aerodynamic load factor in turns and scale on ARSPD_FBW_MIN that must be set correctly. Without airspeed sensor, uses synthetic airspeed from wind speed estimate that may both be inaccurate.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    ASCALAR(stall_prevention, "STALL_PREVENTION",  1),

    // @Param: ARSPD_FBW_MIN
    // @DisplayName: Minimum Airspeed
    // 在自动油门模式下所需的最小空速。应设置为比平飞失速速度高出20%
    // @Description: Minimum airspeed demanded in automatic throttle modes. Should be set to 20% higher than level flight stall speed.
    // @Units: m/s
    // @Range: 5 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(airspeed_min, "ARSPD_FBW_MIN",  AIRSPEED_FBW_MIN),

    // @Param: ARSPD_FBW_MAX
    // @DisplayName: Maximum Airspeed
    // 在自动油门模式下所需的最大空速。应设置为略低于在最大油门（THR_MAX）时的平飞速度，并且至少比最小反馈空速（ARSPD_FBW_MIN）高出50%，
    // 以确保飞行高度控制系统（TECS）能够进行精确的海拔控制
    // @Description: Maximum airspeed demanded in automatic throttle modes. Should be set slightly less than level flight speed at THR_MAX and also at least 50% above ARSPD_FBW_MIN to allow for accurate TECS altitude control.
    // @Units: m/s
    // @Range: 5 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(airspeed_max, "ARSPD_FBW_MAX",  AIRSPEED_FBW_MAX),

    // @Param: FBWB_ELEV_REV
    // @DisplayName: Fly By Wire elevator reverse
    // @Description: Reverse sense of elevator in FBWB and CRUISE modes. When set to 0 up elevator (pulling back on the stick) means to lower altitude. When set to 1, up elevator means to raise altitude.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(flybywire_elev_reverse, "FBWB_ELEV_REV",  0),  //升降舵反向控制标志，为0时，向后拉杆表示降低高度，反之为增加高度

#if AP_TERRAIN_AVAILABLE
    // @Param: TERRAIN_FOLLOW  地形跟随
    // @DisplayName: Use terrain following
    // @Description: This enables terrain following for CRUISE mode, FBWB mode, RTL and for rally points. To use this option you also need to set TERRAIN_ENABLE to 1, which enables terrain data fetching from the GCS, and you need to have a GCS that supports sending terrain data to the aircraft. When terrain following is enabled then CRUISE and FBWB mode will hold height above terrain rather than height above home. In RTL the return to launch altitude will be considered to be a height above the terrain. Rally point altitudes will be taken as height above the terrain. This option does not affect mission items, which have a per-waypoint flag for whether they are height above home or height above the terrain. To use terrain following missions you need a ground station which can set the waypoint type to be a terrain height waypoint when creating the mission.
    // @Values: 0:Disabled,1:Enabled
    // @Bitmask: 0: Enable all modes, 1:FBWB, 2:Cruise, 3:Auto, 4:RTL, 5:Avoid_ADSB, 6:Guided, 7:Loiter, 8:Circle, 9:QRTL, 10:QLand, 11:Qloiter
    // @User: Standard
    GSCALAR(terrain_follow, "TERRAIN_FOLLOW",  0),

    // @Param: TERRAIN_LOOKAHD  指在地形跟随模式下，飞控系统向前查看地形的距离。这个参数用于确定飞控在规划飞行路径时，需要预先考虑多远的前方地形变化，以便及时调整飞行高度，确保无人机能够平稳、安全地飞行
    // @DisplayName: Terrain lookahead
    // @Description: This controls how far ahead the terrain following code looks to ensure it stays above upcoming terrain. A value of zero means no lookahead, so the controller will track only the terrain directly below the aircraft. The lookahead will never extend beyond the next waypoint when in AUTO mode.
    // @Range: 0 10000
    // @Units: m
    // @User: Standard
    GSCALAR(terrain_lookahead, "TERRAIN_LOOKAHD",  2000),
#endif

    // @Param: FBWB_CLIMB_RATE FBWB模式下的爬升率，默认2m/s
    // @DisplayName: Fly By Wire B altitude change rate
    // @Description: This sets the rate in m/s at which FBWB and CRUISE modes will change its target altitude for full elevator deflection. Note that the actual climb rate of the aircraft can be lower than this, depending on your airspeed and throttle control settings. If you have this parameter set to the default value of 2.0, then holding the elevator at maximum deflection for 10 seconds would change the target altitude by 20 meters.
    // @Range: 1 10
    // @Units: m/s
	// @Increment: 0.1
    // @User: Standard
    GSCALAR(flybywire_climb_rate, "FBWB_CLIMB_RATE",  2.0f),

    // @Param: THR_MIN
    // @DisplayName: Minimum Throttle
    // @Description: Minimum throttle percentage used in all modes except manual, provided THR_PASS_STAB is not set. Negative values allow reverse thrust if hardware supports it.
    // @Units: %
    // @Range: -100 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_min,           "THR_MIN",        THROTTLE_MIN),

    // @Param: THR_MAX
    // @DisplayName: Maximum Throttle
    // @Description: Maximum throttle percentage used in all modes except manual, provided THR_PASS_STAB is not set.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_max,           "THR_MAX",        THROTTLE_MAX),

    // @Param: TKOFF_THR_MAX  起飞时允许的最大油门值，如果为0，系统将使用THR_MAX作为起飞时的油门上限
    // @DisplayName: Maximum Throttle for takeoff
    // @Description: The maximum throttle setting during automatic takeoff. If this is zero then THR_MAX is used for takeoff as well.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    ASCALAR(takeoff_throttle_max,   "TKOFF_THR_MAX",        0),

    // @Param: THR_SLEWRATE
    // @DisplayName: Throttle slew rate
    // @Description: Maximum change in throttle percentage per second. Lower limit  based on 1 microsend of servo increase per loop. Divide SCHED_LOOP_RATE by approximately 10 to determine minimum achievable value.
    // @Units: %/s
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_slewrate,      "THR_SLEWRATE",   100),

    // @Param: FLAP_SLEWRATE
    // @DisplayName: Flap slew rate 舵面（特别是副翼或襟翼等）控制输入的变化率限制
    // @Description: maximum percentage change in flap output per second. A setting of 25 means to not change the flap by more than 25% of the full flap range in one second. A value of 0 means no rate limiting.
    // @Units: %/s
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    GSCALAR(flap_slewrate,          "FLAP_SLEWRATE",   75),

    // @Param: THR_SUPP_MAN
    // @DisplayName: Throttle suppress manual passthru
    // 在自动模式下，当油门被抑制时，它通常会被强制归零。如果启用了此选项，那么在油门被抑制期间，它将变为手动油门。这对于汽油发动机很有用，因为在等待起飞时，可以手动保持怠速油门
    // @Description: When throttle is suppressed in auto mode it is normally forced to zero. If you enable this option, then while suppressed it will be manual throttle. This is useful on petrol engines to hold the idle throttle manually while waiting for takeoff
	// @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(throttle_suppress_manual,"THR_SUPP_MAN",   0),

    // @Param: THR_PASS_STAB 自稳模式油门直通
    // @DisplayName: Throttle passthru in stabilize
    // 如果设置了此选项，那么在STABILIZE（自稳）、FBWA（固定翼自动）或ACRO（特技）模式下，油门将直接从遥控器透传。这意味着在这些模式下，THR_MIN（最小油门）和THR_MAX（最大油门）设置将不会被使用。
    // 这对于汽油发动机非常有用，因为你可以设置一个油门切断开关，以抑制低于正常最小值的油门
    // @Description: If this is set then when in STABILIZE, FBWA or ACRO modes the throttle is a direct passthru from the transmitter. This means the THR_MIN and THR_MAX settings are not used in these modes. This is useful for petrol engines where you setup a throttle cut switch that suppresses the throttle below the normal minimum.
	// @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(throttle_passthru_stabilize,"THR_PASS_STAB",   0),

    // @Param: THR_FAILSAFE
    // @DisplayName: Throttle and RC Failsafe Enable
    // 0 表示禁用故障保护功能。1 表示在遥控器（RC）输入丢失时启用故障保护功能。这可以通过油门值低于 THR_FS_VALUE（油门故障保护值）、接收器无法接收到有效脉冲/数据，
    // 或者像 SBUS 这样的接收器提供的 FS 位（故障安全位）被触发来检测。此时，将执行预设的故障保护动作，并且如果此时还有遥控器输入，这些输入将被忽略。
    // 如果设置为 2，则表示当通过上述任何方法检测到遥控器故障保护时，将不会使用遥控器输入，但也不会触发遥控器故障保护动作
    // @Description: 0 disables the failsafe. 1 enables failsafe on loss of RC input. This is detected either by throttle values below THR_FS_VALUE, loss of receiver valid pulses/data, or by the FS bit in receivers that provide it, like SBUS. A programmable failsafe action will occur and RC inputs, if present, will be ignored. A value of 2 means that the RC inputs won't be used when RC failsafe is detected by any of the above methods, but it won't trigger an RC failsafe action.
    // @Values: 0:Disabled,1:Enabled,2:EnabledNoFailsafe
    // @User: Standard
    GSCALAR(throttle_fs_enabled,    "THR_FAILSAFE",   int(ThrFailsafe::Enabled)),


    // @Param: THR_FS_VALUE
    // @DisplayName: Throttle Failsafe Value  油门故障保护值
    // @Description: The PWM level on the throttle input channel below which throttle failsafe triggers. Note that this should be well below the normal minimum for your throttle channel.
    // @Range: 925 2200
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_fs_value,      "THR_FS_VALUE",   950),

    // @Param: TRIM_THROTTLE
    // @DisplayName: Throttle cruise percentage
    // 在自动油门模式下飞行时应用的油门目标百分比，以及维持TRIM_ARSPD_CM所需的油门百分比。注意：在飞行结束时，如果电池电压较低，可能需要更高的油门来维持空速
    // @Description: Target percentage of throttle to apply for flight in automatic throttle modes and throttle percentage that maintains TRIM_ARSPD_CM. Caution: low battery voltages at the end of flights may require higher throttle to maintain airspeed.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_cruise,        "TRIM_THROTTLE",  THROTTLE_CRUISE),

    // @Param: THROTTLE_NUDGE
    // @DisplayName: Throttle nudge enable 油门微调使能
    // 启用后，此功能将在自动油门模式下使用油门输入来“微调”油门或空速，使其增加或减少。当您有空速传感器时，微调会影响目标空速，因此，油门输入超过50%时，目标空速将从TRIM_ARSPD_CM增加至最大值ARSPD_FBW_MAX。
    // 当没有启用空速传感器时，油门微调将在油门输入超过50%时提高目标油门
    // @Description: When enabled, this uses the throttle input in auto-throttle modes to 'nudge' the throttle or airspeed to higher or lower values. When you have an airspeed sensor the nudge affects the target airspeed, so that throttle inputs above 50% will increase the target airspeed from TRIM_ARSPD_CM up to a maximum of ARSPD_FBW_MAX. When no airspeed sensor is enabled the throttle nudge will push up the target throttle for throttle inputs above 50%.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(throttle_nudge,         "THROTTLE_NUDGE",  1),

    // @Param: FS_SHORT_ACTN
    // @DisplayName: Short failsafe action 用于定义当发生短故障安全事件（short failsafe event）时，系统应该采取的应对措施
    // @Description: The action to take on a short (FS_SHORT_TIMEOUT) failsafe event. A short failsafe event can be triggered either by loss of RC control (see THR_FS_VALUE) or by loss of GCS control (see FS_GCS_ENABL). If in CIRCLE or RTL mode this parameter is ignored. A short failsafe event in stabilization and manual modes will cause a change to CIRCLE mode if FS_SHORT_ACTN is 0 or 1, a change to FBWA mode with zero throttle if FS_SHORT_ACTN is 2, and a change to FBWB mode if FS_SHORT_ACTN is 4. In all other modes (AUTO, GUIDED and LOITER) a short failsafe event will cause no mode change if FS_SHORT_ACTN is set to 0, will cause a change to CIRCLE mode if set to 1, will change to FBWA mode with zero throttle if set to 2, or will change to FBWB if set to 4. Please see the documentation for FS_LONG_ACTN for the behaviour after FS_LONG_TIMEOUT seconds of failsafe.
    // @Values: 0:CIRCLE/no change(if already in AUTO|GUIDED|LOITER),1:CIRCLE,2:FBWA at zero throttle,3:Disable,4:FBWB
    // @User: Standard
    GSCALAR(fs_action_short,        "FS_SHORT_ACTN",  FS_ACTION_SHORT_BESTGUESS),

    // @Param: FS_SHORT_TIMEOUT
    // @DisplayName: Short failsafe timeout 用于定义短故障安全事件（short failsafe event）的触发时间阈值
    // @Description: The time in seconds that a failsafe condition has to persist before a short failsafe event will occur. This defaults to 1.5 seconds
    // @Units: s
    // @Range: 1 100
    // @Increment: 0.5
    // @User: Standard
    GSCALAR(fs_timeout_short,        "FS_SHORT_TIMEOUT", 1.5f),

    // @Param: FS_LONG_ACTN
    // @DisplayName: Long failsafe action 
    // 在发生长时间（FS_LONG_TIMEOUT 秒）的故障安全模式事件时，应采取以下行动。
    // 如果故障安全模式启动时飞机处于稳定模式或手动模式，并且发生了长时间故障安全事件，那么：
    // 如果 FS_LONG_ACTN 设置为 0 或 1，飞机将切换到 RTL（返回起飞点）模式。
    // 如果 FS_LONG_ACTN 设置为 2，飞机将切换到 FBWA（全备份自动）模式。

    // 如果故障安全模式启动时飞机处于自动模式（如 AUTO 或 GUIDED），那么：
    // 如果 FS_LONG_ACTN 设置为 0，飞机将继续保持在当前的自动模式。
    // 如果 FS_LONG_ACTN 设置为 1，飞机将切换到 RTL 模式。
    // 如果 FS_LONG_ACTN 设置为 2，飞机将切换到 FBWA 模式
    // 如果 FS_LONG_ACTN 设置为 3，飞机将部署降落伞（请确保降落伞已配置并启用）
    // @Description: The action to take on a long (FS_LONG_TIMEOUT seconds) failsafe event. If the aircraft was in a stabilization or manual mode when failsafe started and a long failsafe occurs then it will change to RTL mode if FS_LONG_ACTN is 0 or 1, and will change to FBWA if FS_LONG_ACTN is set to 2. If the aircraft was in an auto mode (such as AUTO or GUIDED) when the failsafe started then it will continue in the auto mode if FS_LONG_ACTN is set to 0, will change to RTL mode if FS_LONG_ACTN is set to 1 and will change to FBWA mode if FS_LONG_ACTN is set to 2. If FS_LONG_ACTION is set to 3, the parachute will be deployed (make sure the chute is configured and enabled). 
    // @Values: 0:Continue,1:ReturnToLaunch,2:Glide,3:Deploy Parachute
    // @User: Standard
    GSCALAR(fs_action_long,         "FS_LONG_ACTN",   FS_ACTION_LONG_CONTINUE),

    // @Param: FS_LONG_TIMEOUT
    // @DisplayName: Long failsafe timeout
    // @Description: The time in seconds that a failsafe condition has to persist before a long failsafe event will occur. This defaults to 5 seconds.
    // @Units: s
    // @Range: 1 300
    // @Increment: 0.5
    // @User: Standard
    GSCALAR(fs_timeout_long,        "FS_LONG_TIMEOUT", 5),

    // @Param: FS_GCS_ENABL
    // @DisplayName: GCS failsafe enable
    // 启用地面控制站遥测故障安全模式。如果在FS_LONG_TIMEOUT秒内未收到MAVLink心跳消息，将触发故障安全模式。有三种可能的启用设置。
    // 将FS_GCS_ENABL设置为1: 当飞行器未收到MAVLink心跳消息时，将触发地面控制站故障安全模式。
    // 将FS_GCS_ENABL设置为2: 在丢失心跳消息或收到来自启用MAVLink的3DR无线电的RADIO_STATUS消息（表明地面站未从飞行器接收状态更新，这由RADIO_STATUS.remrssi字段为零指示，这种情况可能因地面站和飞行器无线电的不对称噪声导致单向链路而出现）时，将触发地面控制站故障安全模式。
    // 将FS_GCS_ENABL设置为3: 仅在自动（AUTO）模式下，心跳消息丢失（如选项1）会触发地面控制站故障安全模式。
    // 警告：启用此选项可能导致飞机在失去与地面站的联系后进入故障安全模式并在地面上运行发动机。如果在电动飞机上启用此选项，则应同时启用ARMING_REQUIRED（解锁要求）。
    // @Description: Enable ground control station telemetry failsafe. Failsafe will trigger after FS_LONG_TIMEOUT seconds of no MAVLink heartbeat messages. There are three possible enabled settings. Setting FS_GCS_ENABL to 1 means that GCS failsafe will be triggered when the aircraft has not received a MAVLink HEARTBEAT message. Setting FS_GCS_ENABL to 2 means that GCS failsafe will be triggered on either a loss of HEARTBEAT messages, or a RADIO_STATUS message from a MAVLink enabled 3DR radio indicating that the ground station is not receiving status updates from the aircraft, which is indicated by the RADIO_STATUS.remrssi field being zero (this may happen if you have a one way link due to asymmetric noise on the ground station and aircraft radios).Setting FS_GCS_ENABL to 3 means that GCS failsafe will be triggered by Heartbeat(like option one), but only in AUTO mode. WARNING: Enabling this option opens up the possibility of your plane going into failsafe mode and running the motor on the ground it it loses contact with your ground station. If this option is enabled on an electric plane then you should enable ARMING_REQUIRED.
    // @Values: 0:Disabled,1:Heartbeat,2:HeartbeatAndREMRSSI,3:HeartbeatAndAUTO
    // @User: Standard
    GSCALAR(gcs_heartbeat_fs_enabled, "FS_GCS_ENABL", GCS_FAILSAFE_OFF),

    // @Param: FLTMODE_CH
    // @DisplayName: Flightmode channel  用于指定遥控器上哪个通道（Channel）用于控制飞行模式的切换
    // @Description: RC Channel to use for flight mode control
    // @Range: 1 16
    // @Increment: 1
    // @User: Advanced
    GSCALAR(flight_mode_channel,    "FLTMODE_CH",     FLIGHT_MODE_CHANNEL),

    // @Param: FLTMODE1
    // @DisplayName: FlightMode1
    // @Description: Flight mode for switch position 1 (910 to 1230 and above 2049)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,4:ACRO,5:FBWA,6:FBWB,7:CRUISE,8:AUTOTUNE,10:Auto,11:RTL,12:Loiter,13:TAKEOFF,14:AVOID_ADSB,15:Guided,17:QSTABILIZE,18:QHOVER,19:QLOITER,20:QLAND,21:QRTL,22:QAUTOTUNE,23:QACRO,24:THERMAL,25:Loiter to QLand
    // @User: Standard
    GSCALAR(flight_mode1,           "FLTMODE1",       FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: FlightMode2
    // @Description: Flight mode for switch position 2 (1231 to 1360)
    // @CopyValuesFrom: FLTMODE1
    // @User: Standard
    GSCALAR(flight_mode2,           "FLTMODE2",       FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: FlightMode3
    // @Description: Flight mode for switch position 3 (1361 to 1490)
    // @CopyValuesFrom: FLTMODE1
    // @User: Standard
    GSCALAR(flight_mode3,           "FLTMODE3",       FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: FlightMode4
    // @Description: Flight mode for switch position 4 (1491 to 1620)
    // @CopyValuesFrom: FLTMODE1
    // @User: Standard
    GSCALAR(flight_mode4,           "FLTMODE4",       FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: FlightMode5
    // @Description: Flight mode for switch position 5 (1621 to 1749)
    // @CopyValuesFrom: FLTMODE1
    // @User: Standard
    GSCALAR(flight_mode5,           "FLTMODE5",       FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: FlightMode6
    // @Description: Flight mode for switch position 6 (1750 to 2049)
    // @CopyValuesFrom: FLTMODE1
    // @User: Standard
    GSCALAR(flight_mode6,           "FLTMODE6",       FLIGHT_MODE_6),

    // @Param: INITIAL_MODE
    // @DisplayName: Initial flight mode
    // 选择启动时进入的模式。当你想要在没有接收器的情况下以自动模式启动时，这非常有用
    // @Description: This selects the mode to start in on boot. This is useful for when you want to start in AUTO mode on boot without a receiver.
    // @CopyValuesFrom: FLTMODE1
    // @User: Advanced
    GSCALAR(initial_mode,        "INITIAL_MODE",     Mode::Number::MANUAL),

    // @Param: LIM_ROLL_CD
    // @DisplayName: Maximum Bank Angle
    // 在具有稳定限制的模式下，所指定的最大倾斜角（滚转角）。增加此值可实现更急剧的转弯，但减小此值可防止加速失速
    // 增加roll_limit_cd的值：这将允许飞行器在转弯时以更大的角度倾斜，从而实现更急剧的转弯。这对于需要快速改变方向或执行特技飞行的场景可能很有用。
    // 减小roll_limit_cd的值：这将限制飞行器在转弯时的倾斜角度，有助于防止因过度倾斜而导致的加速失速。这对于确保飞行稳定性和安全性尤为重要。
    // @Description: Maximum bank angle commanded in modes with stabilized limits. Increase this value for sharper turns, but decrease to prevent accelerated stalls.
    // @Units: cdeg
    // @Range: 0 9000
    // @Increment: 10
    // @User: Standard
    ASCALAR(roll_limit_cd,          "LIM_ROLL_CD",    HEAD_MAX_CENTIDEGREE),

    // @Param: LIM_PITCH_MAX 增加这个参数的值可以让无人机在转弯时更加急剧，因为俯仰角的增加会提高无人机的爬升或下降率，从而使其能够在更短的时间内完成转弯。然而，过高的俯仰角限制可能会导致无人机在高速飞行时遇到空气动力学问题，如加速失速，这会增加飞行的不稳定性和风险
    // @DisplayName: Maximum Pitch Angle
    // @Description: Maximum pitch up angle commanded in modes with stabilized limits.
    // @Units: cdeg
    // @Range: 0 9000
    // @Increment: 10
    // @User: Standard
    ASCALAR(pitch_limit_max_cd,     "LIM_PITCH_MAX",  PITCH_MAX_CENTIDEGREE),

    // @Param: LIM_PITCH_MIN
    // @DisplayName: Minimum Pitch Angle
    // @Description: Maximum pitch down angle commanded in modes with stabilized limits
    // @Units: cdeg
    // @Range: -9000 0
    // @Increment: 10
    // @User: Standard
    ASCALAR(pitch_limit_min_cd,     "LIM_PITCH_MIN",  PITCH_MIN_CENTIDEGREE),

    // @Param: ACRO_ROLL_RATE
    // @DisplayName: ACRO mode roll rate 
    // ACRO_ROLL_RATE参数控制了在Acro（手动）模式下，当用户的摇杆全偏转时，无人飞行器可以达到的最大滚动率（roll rate）。它决定了飞行器在Acro模式下响应滚动输入时的敏捷度和速度
    // Acro模式：在Acro模式下，飞行器的飞行姿态和速度完全由用户通过遥控器控制。这意味着飞行器不会自动稳定或修正姿态，用户需要直接控制滚转、俯仰和偏航来操纵飞行器
    // @Description: The maximum roll rate at full stick deflection in ACRO mode
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    GSCALAR(acro_roll_rate,          "ACRO_ROLL_RATE",    180),

    // @Param: ACRO_PITCH_RATE
    // @DisplayName: ACRO mode pitch rate
    // @Description: The maximum pitch rate at full stick deflection in ACRO mode
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    GSCALAR(acro_pitch_rate,          "ACRO_PITCH_RATE",  180),

    // @Param: ACRO_YAW_RATE
    // @DisplayName: ACRO mode yaw rate
    // @Description: The maximum yaw rate at full stick deflection in ACRO mode. If this is zero then rudder is directly controlled by rudder stick input. This option is only available if you also set YAW_RATE_ENABLE to 1.
    // @Units: deg/s
    // @Range: 0 500
    // @Increment: 1
    // @User: Standard
    GSCALAR(acro_yaw_rate,            "ACRO_YAW_RATE",    0),
    
    // @Param: ACRO_LOCKING
    // @DisplayName: ACRO mode attitude locking 当遥控器的操纵杆（sticks）被释放（即回到中立位置）时，启用飞行器的姿态锁定功能
    // @Description: Enable attitude locking when sticks are released
    // @Values: 0:Disabled,1:Enabled,2:Quaternion(四元数)
    // @User: Standard
    GSCALAR(acro_locking,             "ACRO_LOCKING",     0),

    // @Param: GROUND_STEER_ALT
    // @DisplayName: Ground steer altitude
    // 表示在低于或等于此设定高度（相对于起飞点或某个参考点）时，无人机将使用地面转向控制器来控制方向舵。这种控制方式通常用于低空飞行或着陆过程中，以提供更精确和稳定的地面转向控制
    // 如果ground_steer_alt参数被设置为一个非零值，那么当无人机的高度低于或等于这个设定值时，Ardupilot将使用STEER2SRV控制器或其他类似的地面转向控制机制来控制方向舵。这有助于无人机在接近地面时保持稳定的航向和位置控制
    // @Description: Altitude at which to use the ground steering controller on the rudder. If non-zero then the STEER2SRV controller will be used to control the rudder for altitudes within this limit of the home altitude.
    // @Units: m
    // @Range: -100 100
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(ground_steer_alt,         "GROUND_STEER_ALT",   0),

    // @Param: GROUND_STEER_DPS
    // @DisplayName: Ground steer rate 用于控制无人机（或地面车辆）在地面行驶时的转向速率
    // @Description: Ground steering rate in degrees per second for full rudder stick deflection 完全舵杆偏转的地面转向率（以度/秒为单位）
    // @Units: deg/s
    // @Range: 10 360
    // @Increment: 1
    // @User: Advanced
    GSCALAR(ground_steer_dps,         "GROUND_STEER_DPS",  90),

    // @Param: MIXING_GAIN
    // @DisplayName: Mixing Gain
    // V尾翼和升降舵输出混合器的增益。默认值为0.5，这确保了混合器不会饱和，允许两个输入通道都达到极限，同时保持对输出的控制。硬件混合器通常具有1.0的增益，这提供了更大的舵机行程，
    // 但可能会导致饱和。如果在使用VTAIL_OUTPUT或ELEVON_OUTPUT功能时，您的舵机行程不够，那么您可以通过MIXING_GAIN来增加增益。该混合器允许输出范围在900到2100微秒之间
    // @Description: The gain for the Vtail and elevon output mixers. The default is 0.5, which ensures that the mixer doesn't saturate, allowing both input channels to go to extremes while retaining control over the output. Hardware mixers often have a 1.0 gain, which gives more servo throw, but can saturate. If you don't have enough throw on your servos with VTAIL_OUTPUT or ELEVON_OUTPUT enabled then you can raise the gain using MIXING_GAIN. The mixer allows outputs in the range 900 to 2100 microseconds.
    // @Range: 0.5 1.2
    // @User: Standard
    GSCALAR(mixing_gain,            "MIXING_GAIN",    0.5f),

    // @Param: RUDDER_ONLY 仅用方向舵
    // @DisplayName: Rudder only aircraft
    // 启用仅方向舵模式。在姿态控制模式（如FBWA）下，方向舵将控制飞行姿态。您应该设置您的遥控器，将滚转摇杆输入发送到RCMAP_YAW通道（通常是通道4）。
    // 方向舵舵机也应连接到RCMAP_YAW通道。请注意，对于仅使用方向舵的飞行器，自动地面转向将被禁用。您还应将KFF_RDDRMIX设置为1.0。此外，您需要为您的飞行器适当设置YAW2SRV_DAMP（偏航到舵机阻尼）参数。YAW2SRV_DAMP的一个良好起点值是0.5
    // @Description: Enable rudder only mode. The rudder will control attitude in attitude controlled modes (such as FBWA). You should setup your transmitter to send roll stick inputs to the RCMAP_YAW channel (normally channel 4). The rudder servo should be attached to the RCMAP_YAW channel as well. Note that automatic ground steering will be disabled for rudder only aircraft. You should also set KFF_RDDRMIX to 1.0. You will also need to setup the YAW2SRV_DAMP yaw damping appropriately for your aircraft. A value of 0.5 for YAW2SRV_DAMP is a good starting point.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(rudder_only,             "RUDDER_ONLY",  0),

    // @Param: MIXING_OFFSET
    // @DisplayName: Mixing Offset
    // V型尾翼和升降舵输出混合器的偏移量，以百分比表示。该参数可以与MIXING_GAIN（混合增益）结合使用，以配置控制面对输入的响应方式。通过设置此参数为正值或负值，可以增加对副翼或升降舵输入的响应。
    // 一个常见的用途是输入正值，以增加飞翼上升降舵对副翼输入的响应。默认值为零，意味着副翼输入响应与升降舵输入响应相等
    // @Description: The offset for the Vtail and elevon output mixers, as a percentage. This can be used in combination with MIXING_GAIN to configure how the control surfaces respond to input. The response to aileron or elevator input can be increased by setting this parameter to a positive or negative value. A common usage is to enter a positive value to increase the aileron response of the elevons of a flying wing. The default value of zero will leave the aileron-input response equal to the elevator-input response.
    // @Units: d%
    // @Range: -1000 1000
    // @User: Standard
    GSCALAR(mixing_offset,          "MIXING_OFFSET",  0),

    // @Param: DSPOILR_RUD_RATE
    // @DisplayName: Differential spoilers rudder rate
    // 设置舵面输出将应用于差速扰流板的偏转量，以百分比表示。默认值为100，意味着满舵将导致差速扰流板满偏转。如果设置为0，则差速扰流板将完全跟随升降舵的动作（无舵面效果）
    // @Description: Sets the amount of deflection that the rudder output will apply to the differential spoilers, as a percentage. The default value of 100 results in full rudder applying full deflection. A value of 0 will result in the differential spoilers exactly following the elevons (no rudder effect).
    // @Units: %
    // @Range: -100 100
    // @User: Standard
    GSCALAR(dspoiler_rud_rate,      "DSPOILR_RUD_RATE",  DSPOILR_RUD_RATE_DEFAULT),

    // @Param: SYS_NUM_RESETS 系统复位次数
    // @DisplayName: Num Resets
    // @Description: Number of APM board resets
    // @ReadOnly: True
    // @User: Advanced
    GSCALAR(num_resets,             "SYS_NUM_RESETS", 0),

    // @Param: LOG_BITMASK 控制数据闪存中记录的消息类型。这个参数的设置决定了哪些数据会被记录到飞行控制器的日志中，以便于后续的数据分析和故障排查
    // @DisplayName: Log bitmask
    // 这是一个关于启用哪些机载日志类型的位图（Bitmap）。该值由您希望保存的每种日志类型的数值之和组成。通常，通过将这个值设置为65535来启用所有基本日志类型，这是最佳做法
    // @Description: Bitmap of what on-board log types to enable. This value is made up of the sum of each of the log types you want to be saved. It is usually best just to enable all basic log types by setting this to 65535.
    // @Bitmask: 0:Fast Attitude,1:Medium Attitude,2:GPS,3:Performance,4:Control Tuning,5:Navigation Tuning,7:IMU,8:Mission Commands,9:Battery Monitor,10:Compass,11:TECS,12:Camera,13:RC Input-Output,14:Rangefinder,19:Raw IMU,20:Fullrate Attitude,21:Video Stabilization
    // @User: Advanced
    GSCALAR(log_bitmask,            "LOG_BITMASK",    DEFAULT_LOG_BITMASK),

    // @Param: TRIM_ARSPD_CM
    // @DisplayName: Target airspeed 
    // @Description: Target airspeed in cm/s in automatic throttle modes. Value is as an indicated (calibrated/apparent) airspeed. 在自动油门模式下，目标空速以厘米每秒（cm/s）为单位。该值表示指示空速（校准/表观空速）
    // @Units: cm/s
    // @User: Standard
    ASCALAR(airspeed_cruise_cm,     "TRIM_ARSPD_CM",  AIRSPEED_CRUISE_CM),

    // @Param: SCALING_SPEED 
    // @DisplayName: speed used for speed scaling calculations
    // 在计算舵面速度缩放比例时使用的空速（单位：米/秒）。请注意，更改此值将影响所有PID（比例-积分-微分）控制器的值。
    // @Description: Airspeed in m/s to use when calculating surface speed scaling. Note that changing this value will affect all PID values
    // @Units: m/s
    // @Range: 0 50
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(scaling_speed,        "SCALING_SPEED",    SCALING_SPEED),

    // @Param: MIN_GNDSPD_CM 
    // @DisplayName: Minimum ground speed
    // @Description: Minimum ground speed in cm/s when under airspeed control 在空速控制下，最小地面速度（厘米/秒）
    // @Units: cm/s
    // @User: Advanced
    ASCALAR(min_gndspeed_cm,      "MIN_GNDSPD_CM",  MIN_GNDSPEED_CM),

    // @Param: TRIM_PITCH_CD
    // 用于调整飞机的俯仰平衡，以便在飞行中保持稳定的姿态。这里的“cd”可能代表厘度（centidegree），即一百分之一度，这是一个常用于描述飞行器姿态调整的小角度单位。
    // 通过调整pitch_trim_cd参数，飞行员或地面站可以微调飞机的俯仰姿态，以补偿因风、负载变化或其他因素导致的姿态偏差。这种调整有助于确保飞机在飞行过程中能够保持期望的姿态和飞行路径
    // @DisplayName: Pitch angle offset
    // 对AHRS俯仰角应用的偏移量，用于飞行中的俯仰微调。正确的地面调平比更改此参数更好
    // @Description: Offset applied to AHRS pitch used for in-flight pitch trimming. Correct ground leveling is better than changing this parameter.
    // @Units: cdeg
    // @Range: -4500 4500
    // @Increment: 10
    // @User: Advanced
    GSCALAR(pitch_trim_cd,        "TRIM_PITCH_CD",  0),

    // @Param: ALT_HOLD_RTL 设定返回起飞点RTL模式时的目标高度
    // @DisplayName: RTL altitude
    // RTL模式下相对于起点的目标高度。如果设置为-1，则保持当前高度。如果飞机不返回起点，则使用集结点的高度
    // @Description: Target altitude above home for RTL mode. Maintains current altitude if set to -1. Rally point altitudes are used if plane does not return to home.
    // @Units: cm
    // @User: Standard
    GSCALAR(RTL_altitude_cm,        "ALT_HOLD_RTL",   ALT_HOLD_HOME_CM),

    // @Param: ALT_HOLD_FBWCM
    // @DisplayName: Minimum altitude for FBWB mode
    // 这是FBWB（固定翼平衡模式）和巡航（CRUISE）模式所允许的最小飞行高度，单位为厘米。如果您试图下降到这个高度以下，飞机会保持水平飞行。如果设置为零，则表示没有限制
    // @Description: This is the minimum altitude in centimeters that FBWB and CRUISE modes will allow. If you attempt to descend below this altitude then the plane will level off. A value of zero means no limit.
    // @Units: cm
    // @User: Standard
    GSCALAR(FBWB_min_altitude_cm,   "ALT_HOLD_FBWCM", ALT_HOLD_FBW_CM),

    // @Param: FLAP_1_PERCNT
    // @DisplayName: Flap 1 percentage
    // 当达到FLAP_1_SPEED时，襟翼位置变化的百分比。使用零来禁用襟翼
    // @Description: The percentage change in flap position when FLAP_1_SPEED is reached. Use zero to disable flaps
    // @Range: 0 100
    // @Increment: 1
    // @Units: %
    // @User: Advanced
    GSCALAR(flap_1_percent,         "FLAP_1_PERCNT",  FLAP_1_PERCENT),

    // @Param: FLAP_1_SPEED
    // @DisplayName: Flap 1 speed
    // 以每秒多少米的速度来启用1%襟翼（FLAP_1_PERCENT）的襟翼展开。请注意，FLAP_1_SPEED（1%襟翼展开速度）应大于或等于FLAP_2_SPEED（另一襟翼展开阶段的速度）
    // @Description: The speed in meters per second at which to engage FLAP_1_PERCENT of flaps. Note that FLAP_1_SPEED should be greater than or equal to FLAP_2_SPEED
    // @Range: 0 100
	// @Increment: 1
    // @Units: m/s
    // @User: Advanced
    GSCALAR(flap_1_speed,           "FLAP_1_SPEED",   FLAP_1_SPEED),

    // @Param: FLAP_2_PERCNT
    // @DisplayName: Flap 2 percentage
    // 当达到FLAP_2_SPEED时，襟翼位置变化的百分比。使用零来禁用襟翼
    // @Description: The percentage change in flap position when FLAP_2_SPEED is reached. Use zero to disable flaps
    // @Range: 0 100
	// @Units: %
    // @Increment: 1
    // @User: Advanced
    GSCALAR(flap_2_percent,         "FLAP_2_PERCNT",  FLAP_2_PERCENT),

    // @Param: FLAP_2_SPEED
    // @DisplayName: Flap 2 speed
    // @Description: The speed in meters per second at which to engage FLAP_2_PERCENT of flaps. Note that FLAP_1_SPEED should be greater than or equal to FLAP_2_SPEED
    // @Range: 0 100
	// @Units: m/s
	// @Increment: 1
    // @User: Advanced
    GSCALAR(flap_2_speed,           "FLAP_2_SPEED",   FLAP_2_SPEED),

#if HAL_WITH_IO_MCU
    // @Param: OVERRIDE_CHAN
    // @DisplayName: IO override channel
    // 如果设置为非零值，则表示在带有IO协处理器的板子上，当主FMU（飞行管理单元）微控制器发生故障时，将使用此RC（遥控）输入通道号进行IO（输入/输出）手动控制。当此RC输入通道的值超过1750时，
    // FMU微控制器将不再参与对舵机的控制，而是由IO微控制器直接控制舵机。请注意，如果FMU因任何原因崩溃，IO手动控制将自动激活。此参数允许您测试手动控制的行为是否正确，而无需实际使FMU崩溃。
    // 此参数可以设置为非零值，用于地面测试目的，或者模拟外部覆盖控制板的效果。请注意，您可以将OVERRIDE_CHAN设置为与FLTMODE_CH相同的通道，以便在飞行模式6下实现基于IO的覆盖控制。
    // 请注意，当因FMU崩溃而触发覆盖控制时，FMU上的6个辅助输出通道将不再更新，因此，您所需的所有飞行控制必须分配给带有IOMCU（输入/输出管理控制单元）的板子上的前8个通道
    // @Description: If set to a non-zero value then this is an RC input channel number to use for giving IO manual control in case the main FMU microcontroller on a board with a IO co-processor fails. When this RC input channel goes above 1750 the FMU microcontroller will no longer be involved in controlling the servos and instead the IO microcontroller will directly control the servos. Note that IO manual control will be automatically activated if the FMU crashes for any reason. This parameter allows you to test for correct manual behaviour without actually crashing the FMU. This parameter is can be set to a non-zero value either for ground testing purposes or for giving the effect of an external override control board. Note that you may set OVERRIDE_CHAN to the same channel as FLTMODE_CH to get IO based override when in flight mode 6. Note that when override is triggered due to a FMU crash the 6 auxiliary output channels on the FMU will no longer be updated, so all the flight controls you need must be assigned to the first 8 channels on boards with an IOMCU.
    // @Range: 0 16
    // @Increment: 1
    // @User: Advanced
    GSCALAR(override_channel,      "OVERRIDE_CHAN",  0),
#endif

    // @Param: RTL_AUTOLAND 用于控制无人机在RTL模式下是否自动着陆的开关
    // @DisplayName: RTL auto land
    // @Description: Automatically begin landing sequence after arriving at RTL location. This requires the addition of a DO_LAND_START mission item, which acts as a marker for the start of a landing sequence. The closest landing sequence will be chosen to the current location. If this is set to 0 and there is a DO_LAND_START mission item then you will get an arming check failure. You can set to a value of 3 to avoid the arming check failure and use the DO_LAND_START for go-around without it changing RTL behaviour. For a value of 1 a rally point will be used instead of HOME if in range (see rally point documentation).
    // @Values: 0:Disable,1:Fly HOME then land,2:Go directly to landing sequence, 3:OnlyForGoAround
    // @User: Standard
    GSCALAR(rtl_autoland,         "RTL_AUTOLAND",   float(RtlAutoland::RTL_DISABLE)),

    // @Param: CRASH_ACC_THRESH
    // @DisplayName: Crash Deceleration Threshold
    // X轴减速阈值用于通知碰撞检测器可能发生了撞击，这有助于在发生撞击后迅速关闭电机。此值应远高于正常飞行期间X轴所承受的正常负向力。请检查飞行日志文件，以确定您的飞行器和电机类型的IMU.x平均值。
    // 数值越高，表示灵敏度越低（在高撞击时才会触发）。对于飞行过程中振动不大的电动飞机，设置值为25较为合适（大约相当于2.5倍重力加速度）。对于汽油或硝基燃料飞机，您可能需要设置更高的值。
    // 若设为0，则会禁用碰撞检测器。
    // @Description: X-Axis deceleration threshold to notify the crash detector that there was a possible impact which helps disarm the motor quickly after a crash. This value should be much higher than normal negative x-axis forces during normal flight, check flight log files to determine the average IMU.x values for your aircraft and motor type. Higher value means less sensative (triggers on higher impact). For electric planes that don't vibrate much during fight a value of 25 is good (that's about 2.5G). For petrol/nitro planes you'll want a higher value. Set to 0 to disable the collision detector.
    // @Units: m/s/s
    // @Range: 10 127
    // @Increment: 1
    // @User: Advanced
    GSCALAR(crash_accel_threshold,          "CRASH_ACC_THRESH",   0),

    // @Param: CRASH_DETECT
    // @DisplayName: Crash Detection
    // 在自动飞行过程中自动检测坠机情况，并执行已选择的位掩码操作。关闭电机（Disarm）将出于安全考虑关闭电机，防止电子调速器（ESC）和电机过热损坏。设置为0以禁用坠机检测功能
    // @Description: Automatically detect a crash during AUTO flight and perform the bitmask selected action(s). Disarm will turn off motor for safety and to help against burning out ESC and motor. Set to 0 to disable crash detection.
    // @Values: 0:Disabled
    // @Bitmask: 0:Disarm
    // @User: Advanced
    ASCALAR(crash_detection_enable,         "CRASH_DETECT",   0),

    // @Group: BARO
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "BARO", AP_Baro),

    // GPS driver
    // @Group: GPS
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS", AP_GPS),

#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera,                  "CAM_", AP_Camera),
#endif

    // @Group: ARMING_
    // @Path: AP_Arming.cpp,../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming,                 "ARMING_", AP_Arming_Plane),

    // @Group: RELAY_
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY_", AP_Relay),

#if PARACHUTE == ENABLED // 降落伞
	// @Group: CHUTE_
    // @Path: ../libraries/AP_Parachute/AP_Parachute.cpp
    GOBJECT(parachute,		"CHUTE_", AP_Parachute),
#endif

    // @Group: RNGFND // 测距仪
    // @Path: ../libraries/AP_RangeFinder/AP_RangeFinder.cpp
    GOBJECT(rangefinder,            "RNGFND", RangeFinder), 

    // @Param: RNGFND_LANDING 这使得能够使用测距仪来实现自动着陆。测距仪将在着陆进近阶段以及最终拉平阶段都会被使用
    // @DisplayName: Enable rangefinder for landing
    // @Description: This enables the use of a rangefinder for automatic landing. The rangefinder will be used both on the landing approach and for final flare
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(rangefinder_landing,    "RNGFND_LANDING",   0),

#if AP_TERRAIN_AVAILABLE
    // @Group: TERRAIN_ 
    // @Path: ../libraries/AP_Terrain/AP_Terrain.cpp
    GOBJECT(terrain,                "TERRAIN_", AP_Terrain), // AP_Terrain是ArduPilot飞行控制系统中一个非常重要的库，它提供了高精度、灵活和可扩展的地形数据处理功能，为无人机的自动着陆、地形跟随和避障等高级飞行任务提供了有力的支持
#endif

#if HAL_ADSB_ENABLED
    // @Group: ADSB_
    // @Path: ../libraries/AP_ADSB/AP_ADSB.cpp
    // AP_ADSB类通常与其他飞行控制类（如Mode类、AP_Arming类等）协同工作，共同实现飞行器的安全、高效飞行。
    // 例如，在飞行器的解锁（arming）过程中，AP_Arming类可能会检查ADS-B相关的故障保护状态，以确保飞行器在解锁前处于安全状态
    GOBJECT(adsb,                "ADSB_", AP_ADSB),

    // @Group: AVD_
    // @Path: ../libraries/AP_Avoidance/AP_Avoidance.cpp
    // AP_Avoidance_Plane是ArduPilot飞行控制系统中用于处理固定翼飞机避障逻辑的类。它主要负责根据飞机的当前状态、位置以及障碍物信息，计算出安全的飞行路径，从而避免与障碍物发生碰撞。
    GOBJECT(avoidance_adsb, "AVD_", AP_Avoidance_Plane),
#endif

#if HAL_QUADPLANE_ENABLED
    // @Group: Q_
    // @Path: quadplane.cpp
    GOBJECT(quadplane,           "Q_", QuadPlane),
#endif

    // @Group: TUNE_
    // @Path: tuning.cpp,../libraries/AP_Tuning/AP_Tuning.cpp    用于固定翼飞机的调谐或参数调整功能
    GOBJECT(tuning,           "TUNE_", AP_Tuning_Plane), 

#if HAL_QUADPLANE_ENABLED
    // @Group: Q_A_
    // @Path: ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp,../libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp
    { AP_PARAM_GROUP, "Q_A_", Parameters::k_param_q_attitude_control,
      (const void *)&plane.quadplane.attitude_control,
      {group_info : AC_AttitudeControl_Multi::var_info}, AP_PARAM_FLAG_POINTER },
#endif

    // @Group: RLL
    // @Path: ../libraries/APM_Control/AP_RollController.cpp
    GOBJECT(rollController,         "RLL",   AP_RollController),

    // @Group: PTCH
    // @Path: ../libraries/APM_Control/AP_PitchController.cpp
    GOBJECT(pitchController,        "PTCH",  AP_PitchController),

    // @Group: YAW
    // @Path: ../libraries/APM_Control/AP_YawController.cpp
    GOBJECT(yawController,          "YAW",   AP_YawController),

    // @Group: STEER2SRV_
    // @Path: ../libraries/APM_Control/AP_SteerController.cpp
	GOBJECT(steerController,        "STEER2SRV_",   AP_SteerController),  // 转向控制器

	// variables not in the g class which contain EEPROM saved variables

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,                "COMPASS_",     Compass),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap,                "RCMAP_",         RCMapper), // 负责处理遥控器（RC）输入信号的映射和转换

    // @Group: SR0_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[0], gcs0,        "SR0_",     GCS_MAVLINK_Parameters),

#if MAVLINK_COMM_NUM_BUFFERS >= 2
    // @Group: SR1_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[1],  gcs1,       "SR1_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 3
    // @Group: SR2_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[2],  gcs2,       "SR2_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 4
    // @Group: SR3_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[3],  gcs3,       "SR3_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 5
    // @Group: SR4_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[4],  gcs4,       "SR4_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 6
    // @Group: SR5_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[5],  gcs5,       "SR5_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 7
    // @Group: SR6_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[6],  gcs6,       "SR6_",     GCS_MAVLINK_Parameters),
#endif

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp  主要负责与惯性导航传感器（如陀螺仪和加速度计）的交互
    GOBJECT(ins,                    "INS_", AP_InertialSensor),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

    // Airspeed was here

    // @Group: NAVL1_
    // @Path: ../libraries/AP_L1_Control/AP_L1_Control.cpp
    GOBJECT(L1_controller,         "NAVL1_",   AP_L1_Control),

    // @Group: TECS_
    // @Path: ../libraries/AP_TECS/AP_TECS.cpp
    GOBJECT(TECS_controller,         "TECS_",   AP_TECS),

#if HAL_MOUNT_ENABLED
    // @Group: MNT
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    // AP_Mount是ArduPilot飞控系统中用于挂载和控制云台设备的类。它提供了一个全面的接口，用于控制和监控云台设备，集成了各种功能，如目标跟踪、模式设置、命令处理等。
    // 通过AP_Mount，飞控系统能够实现对云台设备的精确控制，包括角度调整、速率设定等
    GOBJECT(camera_mount,           "MNT",  AP_Mount),
#endif

    // @Group: LOG
    // @Path: ../libraries/AP_Logger/AP_Logger.cpp
    GOBJECT(logger,           "LOG",  AP_Logger),

    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT", AP_BattMonitor),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    // P_BoardConfig类在Ardupilot的启动和初始化过程中起着至关重要的作用。它负责根据具体的飞控板型号和硬件配置，初始化相关的硬件资源，如串口、GPIO（通用输入输出）引脚、传感器等。
    // 此外，AP_BoardConfig还负责配置一些与硬件相关的参数，以确保飞控系统能够正确识别和使用这些硬件资源
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    // @Group: CAN_
    // @Path: ../libraries/AP_CANManager/AP_CANManager.cpp
    GOBJECT(can_mgr,        "CAN_",       AP_CANManager),
#endif

#if AP_SIM_ENABLED
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp   软件在环仿真
    GOBJECT(sitl, "SIM_", SITL::SIM),
#endif

#if ADVANCED_FAILSAFE == ENABLED
    // @Group: AFS_
    // @Path: ../libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp
    GOBJECT(afs,  "AFS_", AP_AdvancedFailsafe),
#endif

#if AP_OPTICALFLOW_ENABLED
    // @Group: FLOW
    // @Path: ../libraries/AP_OpticalFlow/AP_OpticalFlow.cpp
    GOBJECT(optflow,   "FLOW", AP_OpticalFlow),
#endif

    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECT(mission, "MIS_",       AP_Mission),

    // @Group: RALLY_
    // @Path: ../libraries/AP_Rally/AP_Rally.cpp
    GOBJECT(rally,  "RALLY_",       AP_Rally),

#if HAL_NAVEKF2_AVAILABLE
    // @Group: EK2_
    // @Path: ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    GOBJECTN(ahrs.EKF2, NavEKF2, "EK2_", NavEKF2),
#endif

#if HAL_NAVEKF3_AVAILABLE
    // @Group: EK3_
    // @Path: ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
    GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3),
#endif

    // @Group: RPM
    // @Path: ../libraries/AP_RPM/AP_RPM.cpp
    GOBJECT(rpm_sensor, "RPM", AP_RPM),
    
    // @Group: RSSI_
    // @Path: ../libraries/AP_RSSI/AP_RSSI.cpp
    GOBJECT(rssi, "RSSI_",  AP_RSSI),

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),

    // @Group: 
    // @Path: Parameters.cpp
    GOBJECT(g2, "",  ParametersG2),
    
    // @Group: LAND_
    // @Path: ../libraries/AP_Landing/AP_Landing.cpp
    GOBJECT(landing, "LAND_", AP_Landing),

#if OSD_ENABLED || OSD_PARAM_ENABLED
    // @Group: OSD
    // @Path: ../libraries/AP_OSD/AP_OSD.cpp
    GOBJECT(osd, "OSD", AP_OSD),
#endif

    // @Group: TKOFF_
    // @Path: mode_takeoff.cpp
    GOBJECT(mode_takeoff, "TKOFF_", ModeTakeoff),

    // @Group:
    // @Path: ../libraries/AP_Vehicle/AP_Vehicle.cpp
    { AP_PARAM_GROUP, "", Parameters::k_param_vehicle, (const void *)&plane, {group_info : AP_Vehicle::var_info} },

    AP_VAREND
};

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {

#if HAL_BUTTON_ENABLED
    // @Group: BTN_
    // @Path: ../libraries/AP_Button/AP_Button.cpp
    AP_SUBGROUPPTR(button_ptr, "BTN_", 1, ParametersG2, AP_Button),
#endif

#if AP_ICENGINE_ENABLED
    // @Group: ICE_
    // @Path: ../libraries/AP_ICEngine/AP_ICEngine.cpp
    AP_SUBGROUPINFO(ice_control, "ICE_", 2, ParametersG2, AP_ICEngine),
#endif

    // 3 was used by prototype for servo_channels
    
    // @Param: SYSID_ENFORCE
    // @DisplayName: GCS sysid enforcement
    // @Description: This controls whether packets from other than the expected GCS system ID will be accepted
    // @Values: 0:NotEnforced,1:Enforced
    // @User: Advanced
    AP_GROUPINFO("SYSID_ENFORCE", 4, ParametersG2, sysid_enforce, 0),
#if STATS_ENABLED == ENABLED
    // @Group: STAT
    // @Path: ../libraries/AP_Stats/AP_Stats.cpp
    AP_SUBGROUPINFO(stats, "STAT", 5, ParametersG2, AP_Stats),
#endif

    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    AP_SUBGROUPINFO(servo_channels, "SERVO", 6, ParametersG2, SRV_Channels),

    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    AP_SUBGROUPINFO(rc_channels, "RC", 7, ParametersG2, RC_Channels_Plane),
    
#if HAL_SOARING_ENABLED
    // @Group: SOAR_
    // @Path: ../libraries/AP_Soaring/AP_Soaring.cpp
    AP_SUBGROUPINFO(soaring_controller, "SOAR_", 8, ParametersG2, SoaringController),
#endif
  
    // @Param: RUDD_DT_GAIN
    // @DisplayName: rudder differential thrust gain
    // @Description: gain control from rudder to differential thrust
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RUDD_DT_GAIN", 9, ParametersG2, rudd_dt_gain, 10),

    // @Param: MANUAL_RCMASK
    // @DisplayName: Manual R/C pass-through mask
    // @Description: Mask of R/C channels to pass directly to corresponding output channel when in MANUAL mode. When in any mode except MANUAL the channels selected with this option behave normally. This parameter is designed to allow for complex mixing strategies to be used for MANUAL flight using transmitter based mixing. Note that when this option is used you need to be very careful with pre-flight checks to ensure that the output is correct both in MANUAL and non-MANUAL modes.
    // @Bitmask: 0:Chan1,1:Chan2,2:Chan3,3:Chan4,4:Chan5,5:Chan6,6:Chan7,7:Chan8,8:Chan9,9:Chan10,10:Chan11,11:Chan12,12:Chan13,13:Chan14,14:Chan15,15:Chan16
    // @User: Advanced
    AP_GROUPINFO("MANUAL_RCMASK", 10, ParametersG2, manual_rc_mask, 0),
    
    // @Param: HOME_RESET_ALT
    // @DisplayName: Home reset altitude threshold
    // @Description: When the aircraft is within this altitude of the home waypoint, while disarmed it will automatically update the home position. Set to 0 to continously reset it.
    // @Values: -1:Never reset,0:Always reset
    // @Range: -1 127
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("HOME_RESET_ALT", 11, ParametersG2, home_reset_threshold, 0),

#if GRIPPER_ENABLED == ENABLED
    // @Group: GRIP_
    // @Path: ../libraries/AP_Gripper/AP_Gripper.cpp
    AP_SUBGROUPINFO(gripper, "GRIP_", 12, ParametersG2, AP_Gripper),
#endif

    // @Param: FLIGHT_OPTIONS
    // @DisplayName: Flight mode options
    // @Description: Flight mode specific options
    // @Bitmask: 0:Rudder mixing in direct flight modes only (Manual / Stabilize / Acro),1:Use centered throttle in Cruise or FBWB to indicate trim airspeed, 2:Disable attitude check for takeoff arming, 3:Force target airspeed to trim airspeed in Cruise or FBWB, 4: Climb to ALT_HOLD_RTL before turning for RTL, 5: Enable yaw damper in acro mode, 6: Surpress speed scaling during auto takeoffs to be 1 or less to prevent oscillations without airpseed sensor., 7:EnableDefaultAirspeed for takeoff, 8: Remove the TRIM_PITCH_CD on the GCS horizon, 9: Remove the TRIM_PITCH_CD on the OSD horizon, 10: Adjust mid-throttle to be TRIM_THROTTLE in non-auto throttle modes except MANUAL, 11:Disable suppression of fixed wing rate gains in ground mode, 12: Enable FBWB style loiter altitude control
    // @User: Advanced
    AP_GROUPINFO("FLIGHT_OPTIONS", 13, ParametersG2, flight_options, 0),

#if AP_SCRIPTING_ENABLED
    // @Group: SCR_
    // @Path: ../libraries/AP_Scripting/AP_Scripting.cpp
    AP_SUBGROUPINFO(scripting, "SCR_", 14, ParametersG2, AP_Scripting),
#endif

    // @Param: TKOFF_ACCEL_CNT
    // @DisplayName: Takeoff throttle acceleration count
    // @Description: This is the number of acceleration events to require for arming with TKOFF_THR_MINACC. The default is 1, which means a single forward acceleration above TKOFF_THR_MINACC will arm. By setting this higher than 1 you can require more forward/backward movements to arm.
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("TKOFF_ACCEL_CNT", 15, ParametersG2, takeoff_throttle_accel_count, 1),

#if LANDING_GEAR_ENABLED == ENABLED
    // @Group: LGR_
    // @Path: ../libraries/AP_LandingGear/AP_LandingGear.cpp
    AP_SUBGROUPINFO(landing_gear, "LGR_", 16, ParametersG2, AP_LandingGear),
#endif

    // @Param: DSPOILER_CROW_W1
    // @DisplayName: Differential spoiler crow flaps outer weight
    // @Description: This is amount of deflection applied to the two outer surfaces for differential spoilers for flaps to give crow flaps. It is a number from 0 to 100. At zero no crow flaps are applied. A recommended starting value is 25.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DSPOILER_CROW_W1", 17, ParametersG2, crow_flap_weight_outer, 0),

    // @Param: DSPOILER_CROW_W2
    // @DisplayName: Differential spoiler crow flaps inner weight
    // @Description: This is amount of deflection applied to the two inner surfaces for differential spoilers for flaps to give crow flaps. It is a number from 0 to 100. At zero no crow flaps are applied. A recommended starting value is 45.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DSPOILER_CROW_W2", 18, ParametersG2, crow_flap_weight_inner, 0),

    // @Param: TKOFF_TIMEOUT
    // @DisplayName: Takeoff timeout
    // @Description: This is the timeout for an automatic takeoff. If this is non-zero and the aircraft does not reach a ground speed of at least 4 m/s within this number of seconds then the takeoff is aborted and the vehicle disarmed. If the value is zero then no timeout applies.
    // @Range: 0 120
    // @Increment: 1
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("TKOFF_TIMEOUT", 19, ParametersG2, takeoff_timeout, 0),

    // @Param: DSPOILER_OPTS
    // @DisplayName: Differential spoiler and crow flaps options
    // @Description: Differential spoiler and crow flaps options
    // @Values: 0: none, 1: D spoilers have pitch input, 2: use both control surfaces on each wing for roll, 4: Progressive crow flaps only first (0-50% flap in) then crow flaps (50 - 100% flap in)
    // @Bitmask: 0:pitch control, 1:full span, 2:Progressive crow
    // @User: Advanced
    AP_GROUPINFO("DSPOILER_OPTS", 20, ParametersG2, crow_flap_options, 3),

    // @Param: DSPOILER_AILMTCH
    // @DisplayName: Differential spoiler aileron matching
    // @Description: This scales down the inner flaps so less than full downwards range can be used for differential spoiler and full span ailerons, 100 is use full range, upwards travel is unaffected
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DSPOILER_AILMTCH", 21, ParametersG2, crow_flap_aileron_matching, 100),


    // 22 was EFI

    // @Param: FWD_BAT_VOLT_MAX
    // @DisplayName: Forward throttle battery voltage compensation maximum voltage
    // @Description: Forward throttle battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust). Recommend 4.2 * cell count, 0 = Disabled. Recommend THR_MAX is set to no more than 100 x FWD_BAT_VOLT_MIN / FWD_BAT_VOLT_MAX, THR_MIN is set to no less than -100 x FWD_BAT_VOLT_MIN / FWD_BAT_VOLT_MAX and climb descent rate limits are set accordingly.
    // @Range: 6 35
    // @Units: V
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("FWD_BAT_VOLT_MAX", 23, ParametersG2, fwd_thr_batt_voltage_max, 0.0f),

    // @Param: FWD_BAT_VOLT_MIN
    // @DisplayName: Forward throttle battery voltage compensation minimum voltage
    // @Description: Forward throttle battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust).  Recommend 3.5 * cell count, 0 = Disabled. Recommend THR_MAX is set to no more than 100 x FWD_BAT_VOLT_MIN / FWD_BAT_VOLT_MAX, THR_MIN is set to no less than -100 x FWD_BAT_VOLT_MIN / FWD_BAT_VOLT_MAX and climb descent rate limits are set accordingly.
    // @Range: 6 35
    // @Units: V
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("FWD_BAT_VOLT_MIN", 24, ParametersG2, fwd_thr_batt_voltage_min, 0.0f),

    // @Param: FWD_BAT_IDX
    // @DisplayName: Forward throttle battery compensation index
    // @Description: Which battery monitor should be used for doing compensation for the forward throttle
    // @Values: 0:First battery, 1:Second battery
    // @User: Advanced
    AP_GROUPINFO("FWD_BAT_IDX", 25, ParametersG2, fwd_thr_batt_idx, 0),

    // @Param: FS_EKF_THRESH
    // @DisplayName: EKF failsafe variance threshold
    // @Description: Allows setting the maximum acceptable compass and velocity variance used to check navigation health in VTOL modes
    // @Values: 0.6:Strict, 0.8:Default, 1.0:Relaxed
    // @User: Advanced
    AP_GROUPINFO("FS_EKF_THRESH", 26, ParametersG2, fs_ekf_thresh, FS_EKF_THRESHOLD_DEFAULT),

    // @Param: RTL_CLIMB_MIN
    // @DisplayName: RTL minimum climb
    // @Description: The vehicle will climb this many m during the initial climb portion of the RTL. During this time the roll will be limited to LEVEL_ROLL_LIMIT degrees.
    // @Units: m
    // @Range: 0 30
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RTL_CLIMB_MIN", 27, ParametersG2, rtl_climb_min, 0),

#if OFFBOARD_GUIDED == ENABLED
    // @Group: GUIDED_
    // @Path: ../libraries/AC_PID/AC_PID.cpp
    AP_SUBGROUPINFO(guidedHeading, "GUIDED_", 28, ParametersG2, AC_PID),
#endif // OFFBOARD_GUIDED == ENABLED

    // @Param: MAN_EXPO_ROLL
    // @DisplayName: Manual control expo for roll
    // @Description: Percentage exponential for roll input in MANUAL, ACRO and TRAINING modes
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAN_EXPO_ROLL", 29, ParametersG2, man_expo_roll, 0),

    // @Param: MAN_EXPO_PITCH
    // @DisplayName: Manual input expo for pitch
    // @Description: Percentage exponential for pitch input in MANUAL, ACRO and TRAINING modes
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAN_EXPO_PITCH", 30, ParametersG2, man_expo_pitch, 0),

    // @Param: MAN_EXPO_RUDDER
    // @DisplayName: Manual input expo for rudder
    // @Description: Percentage exponential for rudder input in MANUAL, ACRO and TRAINING modes
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAN_EXPO_RUDDER", 31, ParametersG2, man_expo_rudder, 0),

    // @Param: ONESHOT_MASK
    // @DisplayName: Oneshot output mask
    // @Description: Mask of output channels to use oneshot on
    // @User: Advanced
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15
    AP_GROUPINFO("ONESHOT_MASK", 32, ParametersG2, oneshot_mask, 0),

#if AP_SCRIPTING_ENABLED
    // @Group: FOLL
    // @Path: ../libraries/AP_Follow/AP_Follow.cpp
    AP_SUBGROUPINFO(follow, "FOLL", 33, ParametersG2, AP_Follow),
#endif

    // @Param: FLG_TX_WP_CM
    // @Description: 使能是否发送距下一个航点的距离至地面站并显示在水平仪面板上
    // @Range: 0 -Disable 1 -Enable
    // @User: Standard
    AP_GROUPINFO("FLG_TX_WP_DIS", 34, ParametersG2, flg_send_WP_distance, 0),
    
    AP_GROUPEND
};

ParametersG2::ParametersG2(void) :
    unused_integer{1}
#if AP_ICENGINE_ENABLED
    ,ice_control(plane.rpm_sensor)
#endif
#if HAL_SOARING_ENABLED
    ,soaring_controller(plane.TECS_controller, plane.aparm)
#endif
#if HAL_BUTTON_ENABLED
    ,button_ptr(&plane.button)
#endif
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  This is a conversion table from old parameter values to new
  parameter names. The startup code looks for saved values of the old
  parameters and will copy them across to the new parameters if the
  new parameter does not yet have a saved value. It then saves the new
  value.
  
  Note that this works even if the old parameter has been removed. It
  relies on the old k_param index not being removed
  
  The second column below is the index in the var_info[] table for the
  old object. This should be zero for top level parameters.
 */
static const AP_Param::ConversionInfo conversion_table[] = {
    { Parameters::k_param_log_bitmask_old,    0,      AP_PARAM_INT16, "LOG_BITMASK" },
    { Parameters::k_param_rally_limit_km_old, 0,      AP_PARAM_FLOAT, "RALLY_LIMIT_KM" },
    { Parameters::k_param_rally_total_old,    0,      AP_PARAM_INT8, "RALLY_TOTAL" },
    { Parameters::k_param_serial0_baud,       0,      AP_PARAM_INT16, "SERIAL0_BAUD" },
    { Parameters::k_param_serial1_baud,       0,      AP_PARAM_INT16, "SERIAL1_BAUD" },
    { Parameters::k_param_serial2_baud,       0,      AP_PARAM_INT16, "SERIAL2_BAUD" },

    // these are needed to cope with the change to treat nested index 0 as index 63
    { Parameters::k_param_quadplane,          3,      AP_PARAM_FLOAT, "Q_RT_RLL_P" },
    { Parameters::k_param_quadplane,          4,      AP_PARAM_FLOAT, "Q_RT_PIT_P" },
    { Parameters::k_param_quadplane,          5,      AP_PARAM_FLOAT, "Q_RT_YAW_P" },

    { Parameters::k_param_quadplane,          6,      AP_PARAM_FLOAT, "Q_STB_R_P" },
    { Parameters::k_param_quadplane,          7,      AP_PARAM_FLOAT, "Q_STB_P_P" },
    { Parameters::k_param_quadplane,          8,      AP_PARAM_FLOAT, "Q_STB_Y_P" },

    { Parameters::k_param_quadplane,         12,      AP_PARAM_FLOAT, "Q_PZ_P" },
    { Parameters::k_param_quadplane,         13,      AP_PARAM_FLOAT, "Q_PXY_P" },
    { Parameters::k_param_quadplane,         14,      AP_PARAM_FLOAT, "Q_VXY_P" },
    { Parameters::k_param_quadplane,         15,      AP_PARAM_FLOAT, "Q_VZ_P" },
    { Parameters::k_param_quadplane,         16,      AP_PARAM_FLOAT, "Q_AZ_P" },

    { Parameters::k_param_land_slope_recalc_shallow_threshold,0,AP_PARAM_FLOAT, "LAND_SLOPE_RCALC" },
    { Parameters::k_param_land_slope_recalc_steep_threshold_to_abort,0,AP_PARAM_FLOAT, "LAND_ABORT_DEG" },
    { Parameters::k_param_land_pitch_cd,      0,      AP_PARAM_INT16, "LAND_PITCH_CD" },
    { Parameters::k_param_land_flare_alt,     0,      AP_PARAM_FLOAT, "LAND_FLARE_ALT" },
    { Parameters::k_param_land_flare_sec,     0,      AP_PARAM_FLOAT, "LAND_FLARE_SEC" },
    { Parameters::k_param_land_pre_flare_sec, 0,      AP_PARAM_FLOAT, "LAND_PF_SEC" },
    { Parameters::k_param_land_pre_flare_alt, 0,      AP_PARAM_FLOAT, "LAND_PF_ALT" },
    { Parameters::k_param_land_pre_flare_airspeed, 0, AP_PARAM_FLOAT, "LAND_PF_ARSPD" },
    { Parameters::k_param_land_throttle_slewrate, 0,  AP_PARAM_INT8,  "LAND_THR_SLEW" },
    { Parameters::k_param_land_disarm_delay,  0,      AP_PARAM_INT8,  "LAND_DISARMDELAY" },
    { Parameters::k_param_land_then_servos_neutral,0, AP_PARAM_INT8,  "LAND_THEN_NEUTRAL" },
    { Parameters::k_param_land_abort_throttle_enable,0,AP_PARAM_INT8, "LAND_ABORT_THR" },
    { Parameters::k_param_land_flap_percent,  0,      AP_PARAM_INT8,  "LAND_FLAP_PERCENT" },

    // battery failsafes
    { Parameters::k_param_fs_batt_voltage,    0,      AP_PARAM_FLOAT, "BATT_LOW_VOLT" },
    { Parameters::k_param_fs_batt_mah,        0,      AP_PARAM_FLOAT, "BATT_LOW_MAH" },

    { Parameters::k_param_arming,             3,      AP_PARAM_INT8,  "ARMING_RUDDER" },
    { Parameters::k_param_compass_enabled_deprecated,       0,      AP_PARAM_INT8, "COMPASS_ENABLE" },
    { Parameters::k_param_arming,           128,     AP_PARAM_INT16,  "ARMING_CHECK" },

    { Parameters::k_param_fence_minalt,       0,     AP_PARAM_INT16, "FENCE_ALT_MIN"},
    { Parameters::k_param_fence_maxalt,       0,     AP_PARAM_INT16, "FENCE_ALT_MAX"},
    { Parameters::k_param_fence_retalt,       0,     AP_PARAM_INT16, "FENCE_RET_ALT"},
    { Parameters::k_param_fence_ret_rally,    0,      AP_PARAM_INT8, "FENCE_RET_RALLY"},
    { Parameters::k_param_fence_autoenable,   0,      AP_PARAM_INT8, "FENCE_AUTOENABLE"},
};

struct RCConversionInfo {
    uint16_t old_key; // k_param_*
    uint32_t old_group_element; // index in old object
    RC_Channel::AUX_FUNC fun; // new function
};

static const RCConversionInfo rc_option_conversion[] = {
    { Parameters::k_param_flapin_channel_old, 0, RC_Channel::AUX_FUNC::FLAP},
    { Parameters::k_param_g2, 968, RC_Channel::AUX_FUNC::SOARING},
    { Parameters::k_param_fence_channel, 0, RC_Channel::AUX_FUNC::FENCE},
    { Parameters::k_param_reset_mission_chan, 0, RC_Channel::AUX_FUNC::MISSION_RESET},
    { Parameters::k_param_parachute_channel, 0, RC_Channel::AUX_FUNC::PARACHUTE_RELEASE},
    { Parameters::k_param_fbwa_tdrag_chan, 0, RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER},
    { Parameters::k_param_reset_switch_chan, 0, RC_Channel::AUX_FUNC::MODE_SWITCH_RESET},
};

void Plane::load_parameters(void)
{
    if (!AP_Param::check_var_info()) {
        hal.console->printf("Bad parameter table\n");
        AP_HAL::panic("Bad parameter table");
    }
    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        hal.console->printf("Firmware change: erasing EEPROM...\n");
        StorageManager::erase();
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        hal.console->printf("done.\n");
    }
    g.format_version.set_default(Parameters::k_format_version);

    uint32_t before = micros();
    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
    AP_Param::convert_old_parameters(&conversion_table[0], ARRAY_SIZE(conversion_table));

    // setup defaults in SRV_Channels
    g2.servo_channels.set_default_function(CH_1, SRV_Channel::k_aileron);
    g2.servo_channels.set_default_function(CH_2, SRV_Channel::k_elevator);
    g2.servo_channels.set_default_function(CH_3, SRV_Channel::k_throttle);
    g2.servo_channels.set_default_function(CH_4, SRV_Channel::k_rudder);
        
    SRV_Channels::upgrade_parameters();

#if HAL_QUADPLANE_ENABLED
    if (quadplane.enable) {
        // quadplanes needs a higher loop rate
        AP_Param::set_default_by_name("SCHED_LOOP_RATE", 300);
    }
#endif

    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_PLANE);

    // Convert chan params to RCx_OPTION
    for (uint8_t i=0; i<ARRAY_SIZE(rc_option_conversion); i++) {
        AP_Int8 chan_param;
        AP_Param::ConversionInfo info {rc_option_conversion[i].old_key, rc_option_conversion[i].old_group_element, AP_PARAM_INT8, nullptr};
        if (AP_Param::find_old_parameter(&info, &chan_param) && chan_param.get() > 0) {
            RC_Channel *chan = rc().channel(chan_param.get() - 1);
            if (chan != nullptr && !chan->option.configured()) {
                chan->option.set_and_save((int16_t)rc_option_conversion[i].fun); // save the new param
            }
        }
    }

#if AP_FENCE_ENABLED
    enum ap_var_type ptype_fence_type;
    AP_Int8 *fence_type_new = (AP_Int8*)AP_Param::find("FENCE_TYPE", &ptype_fence_type);
    if (fence_type_new && !fence_type_new->configured()) {
        // If we find the new parameter and it hasn't been configured
        // attempt to upgrade the altitude fences.
        int8_t fence_type_new_val = AC_FENCE_TYPE_POLYGON;
        AP_Int16 fence_alt_min_old;
        AP_Param::ConversionInfo fence_alt_min_info_old = {
            Parameters::k_param_fence_minalt,
            0,
            AP_PARAM_INT16,
            nullptr
        };
        if (AP_Param::find_old_parameter(&fence_alt_min_info_old, &fence_alt_min_old)) {
            if (fence_alt_min_old.configured()) {
                //
                fence_type_new_val |= AC_FENCE_TYPE_ALT_MIN;
            }
        }

        AP_Int16 fence_alt_max_old;
        AP_Param::ConversionInfo fence_alt_max_info_old = {
            Parameters::k_param_fence_maxalt,
            0,
            AP_PARAM_INT16,
            nullptr
        };
        if (AP_Param::find_old_parameter(&fence_alt_max_info_old, &fence_alt_max_old)) {
            if (fence_alt_max_old.configured()) {
                fence_type_new_val |= AC_FENCE_TYPE_ALT_MAX;
            }
        }

        fence_type_new->set_and_save((int8_t)fence_type_new_val);
    }

    AP_Int8 fence_action_old;
    AP_Param::ConversionInfo fence_action_info_old = {
        Parameters::k_param_fence_action,
        0,
        AP_PARAM_INT8,
        "FENCE_ACTION"
    };
    if (AP_Param::find_old_parameter(&fence_action_info_old, &fence_action_old)) {
        enum ap_var_type ptype;
        AP_Int8 *fence_action_new = (AP_Int8*)AP_Param::find(&fence_action_info_old.new_name[0], &ptype);
        uint8_t fence_action_new_val;
        if (fence_action_new && !fence_action_new->configured()) {
            switch(fence_action_old.get()) {
                case 0: // FENCE_ACTION_NONE
                case 2: // FENCE_ACTION_REPORT_ONLY
                default:
                    fence_action_new_val = AC_FENCE_ACTION_REPORT_ONLY;
                    break;
                case 1: // FENCE_ACTION_GUIDED
                    fence_action_new_val = AC_FENCE_ACTION_GUIDED;
                    break;
                case 3: // FENCE_ACTION_GUIDED_THR_PASS
                    fence_action_new_val = AC_FENCE_ACTION_GUIDED_THROTTLE_PASS;
                    break;
                case 4: // FENCE_ACTION_RTL
                    fence_action_new_val = AC_FENCE_ACTION_RTL_AND_LAND;
                    break;
            }
            fence_action_new->set_and_save((int8_t)fence_action_new_val);
            
            // Now upgrade the new fence enable at the same time
            enum ap_var_type ptype_fence_enable;
            AP_Int8 *fence_enable = (AP_Int8*)AP_Param::find("FENCE_ENABLE", &ptype_fence_enable);
            // fences were used if there was a count, and the old fence action was not zero
            AC_Fence *ap_fence = AP::fence();
            bool fences_exist = false;
            if (ap_fence) {
                // If the fence library is present, attempt to read the fence count
                fences_exist = ap_fence->polyfence().total_fence_count() > 0;
            }
            
            bool fences_used = fence_action_old.get() != 0;
            if (fence_enable && !fence_enable->configured()) {
                // The fence enable parameter exists, so now set it accordingly
                fence_enable->set_and_save(fences_exist && fences_used);
            }
        }
    }
#endif // AP_FENCE_ENABLED

#if AP_TERRAIN_AVAILABLE
    g.terrain_follow.convert_parameter_width(AP_PARAM_INT8);
#endif

    g.use_reverse_thrust.convert_parameter_width(AP_PARAM_INT16);


    // PARAMETER_CONVERSION - Added: Oct-2021
#if HAL_EFI_ENABLED
    {
        // Find G2's Top Level Key
        AP_Param::ConversionInfo info;
        if (!AP_Param::find_top_level_key_by_pointer(&g2, info.old_key)) {
            return;
        }

        const uint16_t old_index = 22;       // Old parameter index in g2
        const uint16_t old_top_element = 86; // Old group element in the tree for the first subgroup element (see AP_PARAM_KEY_DUMP)
        AP_Param::convert_class(info.old_key, &efi, efi.var_info, old_index, old_top_element, false);
    }
#endif

#if AP_AIRSPEED_ENABLED
    // PARAMETER_CONVERSION - Added: Jan-2022
    {
        const uint16_t old_key = g.k_param_airspeed;
        const uint16_t old_index = 0;       // Old parameter index in the tree
        const uint16_t old_top_element = 0; // Old group element in the tree for the first subgroup element (see AP_PARAM_KEY_DUMP)
        AP_Param::convert_class(old_key, &airspeed, airspeed.var_info, old_index, old_top_element, true);
    }
#endif

#if HAL_INS_NUM_HARMONIC_NOTCH_FILTERS > 1
    if (!ins.harmonic_notches[1].params.enabled()) {
        // notch filter parameter conversions (moved to INS_HNTC2) for 4.2.x, converted from fixed notch
        const AP_Param::ConversionInfo notchfilt_conversion_info[] {
            { Parameters::k_param_ins, 101, AP_PARAM_INT8,  "INS_HNTC2_ENABLE" },
            { Parameters::k_param_ins, 293, AP_PARAM_FLOAT, "INS_HNTC2_ATT" },
            { Parameters::k_param_ins, 357, AP_PARAM_FLOAT, "INS_HNTC2_FREQ" },
            { Parameters::k_param_ins, 421, AP_PARAM_FLOAT, "INS_HNTC2_BW" },
        };
        uint8_t notchfilt_table_size = ARRAY_SIZE(notchfilt_conversion_info);
        for (uint8_t i=0; i<notchfilt_table_size; i++) {
            AP_Param::convert_old_parameters(&notchfilt_conversion_info[i], 1.0f);
        }
        AP_Param::set_default_by_name("INS_HNTC2_MODE", 0);
        AP_Param::set_default_by_name("INS_HNTC2_HMNCS", 1);
    }
#endif // HAL_INS_NUM_HARMONIC_NOTCH_FILTERS
    
    // PARAMETER_CONVERSION - Added: Mar-2022
#if AP_FENCE_ENABLED
    AP_Param::convert_class(g.k_param_fence, &fence, fence.var_info, 0, 0, true);
#endif

    hal.console->printf("load_all took %uus\n", (unsigned)(micros() - before));
}
