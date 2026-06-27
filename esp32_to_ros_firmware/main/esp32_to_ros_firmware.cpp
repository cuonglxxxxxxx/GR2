#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <time.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <builtin_interfaces/msg/time.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/rmw_microros.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/uart.h"

#include <uros_network_interfaces.h>

extern "C" {
#include "esp32_serial_transport.h"
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
}

#include "config.h"
#include "robot_control.hpp"
#include "robot_encoder.hpp"
#include "robot_pose.hpp"
#include "robot_imu.hpp"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// =============================================================================
// TEST MODE SELECTOR — chon 1 trong cac che do duoi day, flash, monitor
//   0 = production (micro-ROS + control loop binh thuong)
//   1 = TC-ENC-01: verify PPR (motor coast, quay tay 10 vong)
//   2 = TC-IMU-01: live monitor accel + gyro + yaw (kiem tra hanh vi thuc)
//   3 = TC-ENC-03: RPM vs PWM open-loop (BANH XE PHAI NHAC KHOI SAN)
//   4 = TC-PID-01: Kiem thu vong kin PID bang cach tao ma sat
// =============================================================================
#define TC_MODE 0

const float GYRO_VAR   = 1e-4f;
const float ACCEL_VAR  = 4e-2f;

static const char *TAG = "ROBOT_FIRMWARE";

static size_t uart_port = UART_NUM_0;

// Global objects
pcnt_unit_handle_t pcnt_unit_L = NULL, pcnt_unit_R = NULL;
i2c_master_bus_handle_t i2c_bus = NULL;
RobotMotor motorL(PWM_PIN_A, IN_PIN_A, IN_PIN_B, PWM_CHANNEL_L);
RobotMotor motorR(PWM_PIN_B, IN_PIN_C, IN_PIN_D, PWM_CHANNEL_R);
RobotEncoder encoderL(ENCODER_PIN_A_L, ENCODER_PIN_B_L, &pcnt_unit_L);
RobotEncoder encoderR(ENCODER_PIN_A_R, ENCODER_PIN_B_R, &pcnt_unit_R);
RobotIMU imu(I2C_PORT, MPU9250_ADDR);

// PID and Pose
RobotPID pidL(25.0f, 40.0f, 0.1f, PID_DT, ROBOT_INT_MIN, ROBOT_INT_MAX, OUT_MIN, OUT_MAX);
RobotPID pidR(25.0f, 40.0f, 0.1f, PID_DT, ROBOT_INT_MIN, ROBOT_INT_MAX, OUT_MIN, OUT_MAX);
UnicycleOdometry odom(WHEEL_RADIUS, WHEEL_SEPARATION);
EncoderPoseEstimator pose_est(WHEEL_RADIUS, WHEEL_SEPARATION);

static int64_t last_ctrl_time = 0;
static int64_t last_cmd_time = 0;   // Timestamp cmd_vel cuoi — dung de timeout neu Nav2 dung publish

// ROS Entities
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t odom_publisher, imu_publisher;
volatile float rpm_ref_L = 0, rpm_ref_R = 0, w_L_ref = 0, w_R_ref = 0;
volatile float rpm_L = 0, rpm_R = 0;

void setupPins() {
    // 1. Configure PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = PWM_MODE, .duty_resolution = PWM_RESOLUTION,
        .timer_num = PWM_TIMER, .freq_hz = PWM_FREQ, .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    motorL.init(); motorR.init();
    encoderL.init(); encoderR.init();
    pidL.reset(); pidL.setDeadZone(400);
    pidR.reset(); pidR.setDeadZone(400);
    pidL.setFeedforwardParams(15.0f, 100.0f);
    pidR.setFeedforwardParams(15.0f, 100.0f);

    // 2. Configure I2C and IMU
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = (gpio_num_t)I2C_SDA_PIN,
        .scl_io_num = (gpio_num_t)I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = { .enable_internal_pullup = 1 }
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_ADDR,
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev_handle));
    
    imu.setHandle(dev_handle);
    imu.calibrateGyro(500);
}

void init_imu_msg() {
    sensor_msgs__msg__Imu__init(&imu_msg);
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = -1.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    imu_msg.angular_velocity_covariance[0] = GYRO_VAR;
    imu_msg.angular_velocity_covariance[4] = GYRO_VAR;
    imu_msg.angular_velocity_covariance[8] = GYRO_VAR;
    imu_msg.linear_acceleration_covariance[0] = ACCEL_VAR;
    imu_msg.linear_acceleration_covariance[4] = ACCEL_VAR;
    imu_msg.linear_acceleration_covariance[8] = ACCEL_VAR;
    imu_msg.orientation.w = 1.0f;
    imu_msg.orientation.x = 0.0f;
    imu_msg.orientation.y = 0.0f;
    imu_msg.orientation.z = 0.0f;
}

void init_odom_msg() {
    nav_msgs__msg__Odometry__init(&odom_msg);
    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_footprint");
    // Diagonal indices: 0=x, 7=y, 14=z, 21=roll, 28=pitch, 35=yaw
    // Z, roll, pitch are unmeasurable for a 2D robot → very high covariance
    const float MEAS_COV  = 1e-3f;
    const float UNMEAS_COV = 1e9f;
    for (int i = 0; i < 36; ++i) {
        odom_msg.pose.covariance[i]  = 0.0f;
        odom_msg.twist.covariance[i] = 0.0f;
    }
    odom_msg.pose.covariance[0]  = MEAS_COV;   // x
    odom_msg.pose.covariance[7]  = MEAS_COV;   // y
    odom_msg.pose.covariance[14] = UNMEAS_COV; // z
    odom_msg.pose.covariance[21] = UNMEAS_COV; // roll
    odom_msg.pose.covariance[28] = UNMEAS_COV; // pitch
    odom_msg.pose.covariance[35] = MEAS_COV;   // yaw
    odom_msg.twist.covariance[0]  = MEAS_COV;   // vx
    odom_msg.twist.covariance[7]  = MEAS_COV;   // vy
    odom_msg.twist.covariance[14] = UNMEAS_COV; // vz
    odom_msg.twist.covariance[21] = UNMEAS_COV; // roll rate
    odom_msg.twist.covariance[28] = UNMEAS_COV; // pitch rate
    odom_msg.twist.covariance[35] = MEAS_COV;   // yaw rate
}

void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *received_msg = (const geometry_msgs__msg__Twist *)msgin;
    last_cmd_time = esp_timer_get_time();
    // Topic /cmd_vel mang don vi vat ly (m/s, rad/s) tu Nav2.
    // Clamp theo gioi han co hoc cua robot (V_MAX, W_MAX) de bao ve motor.
    float linear  = received_msg->linear.x;
    float angular = received_msg->angular.z;
    if (linear  >  V_MAX) linear  =  V_MAX;
    if (linear  < -V_MAX) linear  = -V_MAX;
    if (angular >  W_MAX) angular =  W_MAX;
    if (angular < -W_MAX) angular = -W_MAX;

    w_L_ref = (linear - (angular * WHEEL_SEPARATION / 2.0f)) / WHEEL_RADIUS;
    w_R_ref = (linear + (angular * WHEEL_SEPARATION / 2.0f)) / WHEEL_RADIUS;
    rpm_ref_L = (w_L_ref * 60.0f) / (2.0f * M_PI);
    rpm_ref_R = (w_R_ref * 60.0f) / (2.0f * M_PI);
}

void timer_callback_ctrl(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) return;
    int64_t now = esp_timer_get_time();
    float dt = (last_ctrl_time == 0) ? 0.05f : (float)(now - last_ctrl_time) / 1000000.0f;
    last_ctrl_time = now;
    if (dt > 0.5f) dt = 0.05f;

    // Watchdog cmd_vel: neu khong nhan cmd moi trong 500ms (Nav2 ket thuc goal,
    // micro-ROS mat ket noi, hoac agent crash) → force stop de tranh runaway.
    if (last_cmd_time > 0 && (now - last_cmd_time) > 500000) {
        w_L_ref = 0.0f; w_R_ref = 0.0f;
        rpm_ref_L = 0.0f; rpm_ref_R = 0.0f;
    }

    rpm_L = encoderL.getRPM(dt);
    rpm_R = encoderR.getRPM(dt);

    imu.update();
    pose_est.update(rpm_L, rpm_R);

    // Encoder-only odometry. Fusion gyro_z + wheel velocity → theta robust
    // lam tren Pi bang robot_localization EKF (Moore & Stouch 2014).
    // ESP32 chi publish raw: /odom (encoder-only) + /imu/data (raw MPU9250).
    odom.update(pose_est.getLinearVelocity(), pose_est.getAngularVelocity(), dt);

    const float EPS_THRESHOLD = 0.01f;
    static float prev_w_L_ref = 0.0f, prev_w_R_ref = 0.0f;
    // Reset PID khi cmd dao chieu — tranh integrator giu lai bias huong cu
    if ((prev_w_L_ref > 0.0f && w_L_ref < 0.0f) || (prev_w_L_ref < 0.0f && w_L_ref > 0.0f)) {
        pidL.reset();
    }
    if ((prev_w_R_ref > 0.0f && w_R_ref < 0.0f) || (prev_w_R_ref < 0.0f && w_R_ref > 0.0f)) {
        pidR.reset();
    }
    prev_w_L_ref = w_L_ref;
    prev_w_R_ref = w_R_ref;

    if (fabsf(w_L_ref) > EPS_THRESHOLD) {
        float uL = fabsf((float)pidL.compute(fabsf(rpm_ref_L), fabsf(rpm_L)));
        motorL.setSpeed(w_L_ref > 0 ? (int)uL : -(int)uL);
    } else { motorL.stop(); pidL.reset(); }

    if (fabsf(w_R_ref) > EPS_THRESHOLD) {
        float uR = fabsf((float)pidR.compute(fabsf(rpm_ref_R), fabsf(rpm_R)));
        motorR.setSpeed(w_R_ref > 0 ? (int)uR : -(int)uR);
    } else { motorR.stop(); pidR.reset(); }
}

void timer_callback_odom(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) return;

    // Lấy timestamp đồng bộ DUY NHẤT cho cả 2 message (Chuẩn micro-ROS Epoch)
    int64_t now_ns = rmw_uros_epoch_nanos();
    builtin_interfaces__msg__Time stamp;
    stamp.sec = (int32_t)(now_ns / 1000000000LL);
    stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

    odom_msg.header.stamp = stamp;
    imu_msg.header.stamp = stamp;

    // Odom message
    odom_msg.pose.pose.position.x = odom.getX();
    odom_msg.pose.pose.position.y = odom.getY();
    float theta = odom.getTheta();
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = sinf(theta / 2.0f);
    odom_msg.pose.pose.orientation.w = cosf(theta / 2.0f);
    odom_msg.twist.twist.linear.x = pose_est.getLinearVelocity();
    odom_msg.twist.twist.angular.z = pose_est.getAngularVelocity();
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

    // IMU message
    imu_msg.linear_acceleration.x = imu.getAccelX();
    imu_msg.linear_acceleration.y = imu.getAccelY();
    imu_msg.linear_acceleration.z = imu.getAccelZ();
    imu_msg.angular_velocity.x = imu.getGyroX();
    imu_msg.angular_velocity.y = imu.getGyroY();
    imu_msg.angular_velocity.z = imu.getGyroZ();
    float current_yaw = imu.getYaw();
    imu_msg.orientation.x = 0.0f;
    imu_msg.orientation.y = 0.0f;
    imu_msg.orientation.z = sinf(current_yaw / 2.0f);
    imu_msg.orientation.w = cosf(current_yaw / 2.0f);
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
}

void setupRos(void * arg) {
    vTaskDelay(pdMS_TO_TICKS(2000));
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(true, (void *)&uart_port, esp32_serial_open, esp32_serial_close, esp32_serial_write, esp32_serial_read);
#endif
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, DOMAIN_ID));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_robot_node", "", &support));
    RCCHECK(rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
    RCCHECK(rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));
    
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
    
    rcl_timer_t timer_ctrl, timer_odom;
    RCCHECK(rclc_timer_init_default(&timer_ctrl, &support, RCL_MS_TO_NS(20), timer_callback_ctrl));
    RCCHECK(rclc_timer_init_default(&timer_odom, &support, RCL_MS_TO_NS(20), timer_callback_odom));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

    // Đợi Agent sẵn sàng và đồng bộ thời gian
    while (rmw_uros_ping_agent(100, 3) != RCL_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    rmw_uros_sync_session(1000);

    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_ctrl));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_odom));

    init_odom_msg();
    init_imu_msg();

    while (1) {
        static int64_t last_sync_ms = 0;
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - last_sync_ms > 10000) {
            if (rmw_uros_ping_agent(10, 1) == RCL_RET_OK) {
                rmw_uros_sync_session(10);
            }
            last_sync_ms = now_ms;
        }

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// [TC-ENC-01] Encoder PPR verification task: in cumulative count moi 500ms.
// Motor giu coast (khong cap PWM) → cho phep quay banh bang tay.
void tc_enc01_task(void *arg) {
    encoderL.resetCount();
    encoderR.resetCount();
    vTaskDelay(pdMS_TO_TICKS(200));
    printf("\n");
    printf("=========================================================\n");
    printf("  [TC-ENC-01] Encoder PPR Verification Mode\n");
    printf("  Target: 10 revolutions = %d pulses (PPR=%d)\n", 10 * ENCODER_PPR, ENCODER_PPR);
    printf("  Pass criteria: |count - 12320| / 12320 < 0.5%%\n");
    printf("  Procedure: quay tay tung banh dung 10 vong forward,\n");
    printf("             doc count cuoi, lap 5 lan, lay trung binh.\n");
    printf("=========================================================\n\n");
    int log_idx = 0;
    while (1) {
        int64_t cnt_L = encoderL.getCount();
        int64_t cnt_R = encoderR.getCount();
        float rev_L = (float)cnt_L / (float)ENCODER_PPR;
        float rev_R = (float)cnt_R / (float)ENCODER_PPR;
        printf("[TC-ENC-01 #%04d] count_L=%7lld (%+7.3f rev)  count_R=%7lld (%+7.3f rev)\n",
               log_idx++, (long long)cnt_L, rev_L, (long long)cnt_R, rev_R);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// [TC-ENC-03] RPM vs PWM open-loop test:
//   - Bypass PID, cap PWM truc tiep tai cac muc co dinh.
//   - Moi muc: cho on dinh 500ms, sample RPM 20 lan x 100ms, in trung binh.
//   - Yeu cau bat buoc: BANH XE NHAC KHOI SAN (no-load), khong dau IMU cung khong sao.
void tc_enc03_task(void *arg) {
    // Quet PWM tu thap (xac nhan deadzone) den cao (xac nhan saturation)
    const int pwm_levels[] = {300, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000};
    const int n_levels = sizeof(pwm_levels) / sizeof(pwm_levels[0]);
    const int stabilize_ms = 500;     // cho motor on dinh sau khi doi PWM
    const int n_samples = 20;          // 20 mau
    const int sample_ms = 100;         // moi 100ms (RPM tinh tren 100ms tot hon 20ms)

    vTaskDelay(pdMS_TO_TICKS(3000));
    printf("\n=========================================================\n");
    printf("  [TC-ENC-03] RPM vs PWM Open-Loop Test\n");
    printf("  PREREQ: !!! NHAC BANH KHOI SAN (no-load) !!!\n");
    printf("  Mode: PWM direct, KHONG PID, KHONG feedforward\n");
    printf("  Per level: %dms stabilize + %d samples x %dms\n",
           stabilize_ms, n_samples, sample_ms);
    printf("  Test starts in 5s ...\n");
    printf("=========================================================\n");
    vTaskDelay(pdMS_TO_TICKS(5000));

    printf("\n  PWM  |   RPM_L  |   RPM_R  | |L-R|/avg (%%)\n");
    printf("-------|----------|----------|---------------\n");

    for (int i = 0; i < n_levels; i++) {
        int pwm = pwm_levels[i];
        motorL.setSpeed(pwm);
        motorR.setSpeed(pwm);

        // Cho motor on dinh, sau do DISCARD count tich luy luc transient
        vTaskDelay(pdMS_TO_TICKS(stabilize_ms));
        encoderL.getRPM(stabilize_ms / 1000.0f);
        encoderR.getRPM(stabilize_ms / 1000.0f);

        float sum_L = 0.0f, sum_R = 0.0f;
        for (int j = 0; j < n_samples; j++) {
            vTaskDelay(pdMS_TO_TICKS(sample_ms));
            float dt = sample_ms / 1000.0f;
            sum_L += encoderL.getRPM(dt);
            sum_R += encoderR.getRPM(dt);
        }
        float avg_L = sum_L / n_samples;
        float avg_R = sum_R / n_samples;
        float avg_abs = (fabsf(avg_L) + fabsf(avg_R)) / 2.0f;
        float diff_pct = (avg_abs > 0.5f) ? fabsf(avg_L - avg_R) / avg_abs * 100.0f : 0.0f;

        printf(" %4d  | %8.2f | %8.2f | %8.2f\n", pwm, avg_L, avg_R, diff_pct);
    }

    motorL.stop();
    motorR.stop();
    printf("\n=== TC-ENC-03 COMPLETE — motor stopped ===\n");
    printf("Reset board de chay lai. Doi TC_MODE=0 truoc khi build production.\n");

    while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

// [TC-IMU-01] IMU live monitor: in du lieu accel/gyro/yaw moi 200ms (5Hz).
// User nghiêng/xoay robot theo cac thao tac chuan, doi chieu voi log de xac
// nhan tung truc doc dung huong va do lon vat ly.
void tc_imu_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    printf("\n=========================================================\n");
    printf("  [TC-IMU-01] IMU Reading Verification (live monitor)\n");
    printf("  Bias gyro Z da hieu chuan trong setupPins() (500 mau).\n");
    printf("  Quy uoc truc IMU (theo firmware):\n");
    printf("    ax: forward (+x)  |  ay: trai (+y)  |  az: len troi (+z)\n");
    printf("    gx,gy,gz: angular velocity, gz = yaw rate (CCW = +)\n");
    printf("    yaw: tich phan gz tu boot, normalize [-pi, +pi]\n");
    printf("  Kich ban kiem tra:\n");
    printf("    1) Dat phang yen     -> a=(0,0,+9.81), g=(0,0,0)\n");
    printf("    2) Nghieng forward   -> ax giam, az tang\n");
    printf("    3) Nghieng sang trai -> ay tang, az giam\n");
    printf("    4) Xoay trai tai cho -> gz > 0 khi quay, yaw tang\n");
    printf("    5) Xoay phai tai cho -> gz < 0, yaw giam\n");
    printf("=========================================================\n\n");

    printf("    ax    |    ay    |    az    |    gx    |    gy    |    gz    |   yaw\n");
    printf("  (m/s^2) |  (m/s^2) |  (m/s^2) |  (rad/s) |  (rad/s) |  (rad/s) |  (rad)\n");
    printf("----------|----------|----------|----------|----------|----------|---------\n");

    while (1) {
        imu.update();
        // printf("  gz(rad/s):%+7.4f\n",imu.getGyroZ());
        printf("  yaw(rad):%+7.3f\n",imu.getYaw());
        vTaskDelay(pdMS_TO_TICKS(800));
    }
}

// [TC-PID-01] PID Closed-loop Monitor: In truc quan Setpoint vs Measured vs PWM.
// De banh xe no-load, sau do dung tay bop vao lop xe de tao ma sat, quan sat PWM tang vọt de bu tru.
void tc_pid_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    printf("\n=========================================================\n");
    printf("  [TC-PID-01] PID Closed-loop Response Test (Banh Trai)\n");
    printf("  Thao tac: \n");
    printf("  1. Robot se chay o toc do: 0.15 m/s (~42.1 RPM)\n");
    printf("  2. Dung tay bop nhe vao banh xe de tao ma sat.\n");
    printf("  3. Quan sat PWM tang len de bu lai luc keo.\n");
    printf("=========================================================\n\n");
    
    // Set toc do co dinh 0.15 m/s di thang
    float linear_cmd = 0.15f; 
    float w_ref = linear_cmd / WHEEL_RADIUS; // rad/s
    float ref_rpm = (w_ref * 60.0f) / (2.0f * M_PI);
    
    int64_t last_time = esp_timer_get_time();
    while (1) {
        int64_t now = esp_timer_get_time();
        float dt = (float)(now - last_time) / 1000000.0f;
        last_time = now;
        if(dt <= 0.001f) dt = 0.05f; // failsafe

        // 1. Doc thuc te
        float act_rpm_L = encoderL.getRPM(dt);
        float act_rpm_R = encoderR.getRPM(dt);

        // 2. Tinh toan PID
        float uL = pidL.compute(ref_rpm, fabsf(act_rpm_L));
        float uR = pidR.compute(ref_rpm, fabsf(act_rpm_R));

        // 3. Xuat ra dong co
        motorL.setSpeed((int)uL);
        motorR.setSpeed((int)uR);

        // 4. Log ra man hinh (giam tan so in de mat nguoi doc duoc)
        static int print_divider = 0;
        if (++print_divider >= 4) { // in ra moi 4*20ms = 80ms
            printf("CaiDat: %5.1f RPM | ThucTe: %5.1f RPM | PWM: %4d\n", 
                   ref_rpm, act_rpm_L, (int)uL);
            print_divider = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Chu ki PID chuan 50Hz (20ms)
    }
}

extern "C" void app_main(void) {
    setupPins();
#if   TC_MODE == 1
    xTaskCreate(tc_enc01_task, "tc_enc01_task", 4096, NULL, 5, NULL);
#elif TC_MODE == 2
    xTaskCreate(tc_imu_task,   "tc_imu_task",   4096, NULL, 5, NULL);
#elif TC_MODE == 3
    xTaskCreate(tc_enc03_task, "tc_enc03_task", 4096, NULL, 5, NULL);
#elif TC_MODE == 4
    xTaskCreate(tc_pid_task,   "tc_pid_task",   4096, NULL, 5, NULL);
#else
    xTaskCreate(setupRos, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
#endif
}
