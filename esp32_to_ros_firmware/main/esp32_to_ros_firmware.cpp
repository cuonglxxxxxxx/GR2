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
#include <builtin_interfaces/msg/time.h>
#include <rosidl_runtime_c/string_functions.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/uart.h"
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
static size_t uart_port = UART_NUM_0;
pcnt_unit_handle_t pcnt_unit_L = NULL, pcnt_unit_R = NULL;
i2c_master_bus_handle_t i2c_bus = NULL;
RobotMotor motorL(PWM_PIN_A, IN_PIN_A, IN_PIN_B, PWM_CHANNEL_L);
RobotMotor motorR(PWM_PIN_B, IN_PIN_C, IN_PIN_D, PWM_CHANNEL_R);
RobotEncoder encoderL(ENCODER_PIN_A_L, ENCODER_PIN_B_L, &pcnt_unit_L);
RobotEncoder encoderR(ENCODER_PIN_A_R, ENCODER_PIN_B_R, &pcnt_unit_R);
RobotIMU imu;
RobotPID pidL(25.0f, 40.0f, 0.1f, PID_DT, ROBOT_INT_MIN, ROBOT_INT_MAX, OUT_MIN, OUT_MAX);
RobotPID pidR(25.0f, 40.0f, 0.1f, PID_DT, ROBOT_INT_MIN, ROBOT_INT_MAX, OUT_MIN, OUT_MAX);
UnicycleOdometry odom;
EncoderPoseEstimator pose_est(WHEEL_RADIUS);
static int64_t last_ctrl_time = 0;
static int64_t last_cmd_time = 0;   
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;
rcl_publisher_t odom_publisher;
volatile float rpm_ref_L = 0, rpm_ref_R = 0;
static float gz_for_odom = 0.0f;
void setupPins() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = PWM_MODE, .duty_resolution = PWM_RESOLUTION,
        .timer_num = PWM_TIMER, .freq_hz = PWM_FREQ, .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    motorL.init(); motorR.init();
    encoderL.init(); encoderR.init();
    pidL.reset(); pidL.setDeadZone(0);
    pidR.reset(); pidR.setDeadZone(0);
    pidL.setFeedforwardParams(13.4f, 2097.0f);
    pidR.setFeedforwardParams(14.1f, 2074.0f);
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
void init_odom_msg() {
    nav_msgs__msg__Odometry__init(&odom_msg);
    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_footprint");
    const float MEAS_COV  = 1e-3f;
    const float UNMEAS_COV = 1e9f;
    for (int i = 0; i < 36; ++i) {
        odom_msg.pose.covariance[i]  = 0.0f;
        odom_msg.twist.covariance[i] = 0.0f;
    }
    odom_msg.pose.covariance[0]  = MEAS_COV;   
    odom_msg.pose.covariance[7]  = MEAS_COV;   
    odom_msg.pose.covariance[14] = UNMEAS_COV; 
    odom_msg.pose.covariance[21] = UNMEAS_COV; 
    odom_msg.pose.covariance[28] = UNMEAS_COV; 
    odom_msg.pose.covariance[35] = MEAS_COV;   
    odom_msg.twist.covariance[0]  = MEAS_COV;   
    odom_msg.twist.covariance[7]  = MEAS_COV;   
    odom_msg.twist.covariance[14] = UNMEAS_COV; 
    odom_msg.twist.covariance[21] = UNMEAS_COV; 
    odom_msg.twist.covariance[28] = UNMEAS_COV; 
    odom_msg.twist.covariance[35] = MEAS_COV;   
}
void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *received_msg = (const geometry_msgs__msg__Twist *)msgin;
    last_cmd_time = esp_timer_get_time();
    float linear  = received_msg->linear.x;
    float angular = received_msg->angular.z;
    if (linear  >  V_MAX) linear  =  V_MAX;
    if (linear  < -V_MAX) linear  = -V_MAX;
    if (angular >  W_MAX) angular =  W_MAX;
    if (angular < -W_MAX) angular = -W_MAX;
    float w_L_ref = (linear - (angular * WHEEL_SEPARATION / 2.0f)) / WHEEL_RADIUS;
    float w_R_ref = (linear + (angular * WHEEL_SEPARATION / 2.0f)) / WHEEL_RADIUS;
    rpm_ref_L = (w_L_ref * 60.0f) / (2.0f * M_PI);
    rpm_ref_R = (w_R_ref * 60.0f) / (2.0f * M_PI);
}
void timer_callback_ctrl(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) return;
    int64_t now = esp_timer_get_time();
    float dt = (last_ctrl_time == 0) ? 0.05f : (float)(now - last_ctrl_time) / 1000000.0f;
    last_ctrl_time = now;
    if (dt > 0.5f) dt = 0.05f;
    if (last_cmd_time > 0 && (now - last_cmd_time) > 500000) {
        rpm_ref_L = 0.0f; rpm_ref_R = 0.0f;
    }
    float rpm_L = encoderL.getRPM(dt);
    float rpm_R = encoderR.getRPM(dt);
    imu.update();
    pose_est.update(rpm_L, rpm_R);
    static float gz_bias_rt = 0.0f;
    static bool bias_initialized = false;
    static int still_n = 0;
    static float still_sum = 0.0f;
    bool is_still = (fabsf(rpm_L) < 0.3f && fabsf(rpm_R) < 0.3f);
    float w_L_actual = rpm_L * (2.0f * M_PI / 60.0f);
    float w_R_actual = rpm_R * (2.0f * M_PI / 60.0f);
    float wheel_yaw_rate = WHEEL_RADIUS * (w_R_actual - w_L_actual) / WHEEL_SEPARATION;
    if (is_still) {
        gz_for_odom = 0.0f;
        still_sum += imu.getGyroZ();
        if (++still_n >= 10) {           
            float measured = still_sum / 10.0f;
            if (!bias_initialized) {
                gz_bias_rt = measured;
                bias_initialized = true;
            } else {
                gz_bias_rt = 0.7f * gz_bias_rt + 0.3f * measured;   
            }
            still_n = 0; still_sum = 0.0f;
        }
    } else {
        still_n = 0; still_sum = 0.0f;
        if (fabsf(wheel_yaw_rate) < 0.05f) {              
            gz_bias_rt = 0.995f * gz_bias_rt + 0.005f * imu.getGyroZ();
            bias_initialized = true;
            gz_for_odom = wheel_yaw_rate;
        } else {
            gz_for_odom = imu.getGyroZ() - gz_bias_rt;
        }
    }
    odom.update(pose_est.getLinearVelocity(), gz_for_odom, dt);
    static float prev_rpm_ref_L = 0.0f, prev_rpm_ref_R = 0.0f;
    if ((prev_rpm_ref_L > 0.0f && rpm_ref_L < 0.0f) || (prev_rpm_ref_L < 0.0f && rpm_ref_L > 0.0f)) {
        pidL.reset();
    }
    if ((prev_rpm_ref_R > 0.0f && rpm_ref_R < 0.0f) || (prev_rpm_ref_R < 0.0f && rpm_ref_R > 0.0f)) {
        pidR.reset();
    }
    prev_rpm_ref_L = rpm_ref_L;
    prev_rpm_ref_R = rpm_ref_R;
    motorL.setSpeed(pidL.compute(rpm_ref_L, rpm_L));
    motorR.setSpeed(pidR.compute(rpm_ref_R, rpm_R));
}
void timer_callback_odom(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) return;
    int64_t now_ns = rmw_uros_epoch_nanos();
    builtin_interfaces__msg__Time stamp;
    stamp.sec = (int32_t)(now_ns / 1000000000LL);
    stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);
    odom_msg.header.stamp = stamp;
    odom_msg.pose.pose.position.x = odom.getX();
    odom_msg.pose.pose.position.y = odom.getY();
    float theta = odom.getTheta();
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = sinf(theta / 2.0f);
    odom_msg.pose.pose.orientation.w = cosf(theta / 2.0f);
    odom_msg.twist.twist.linear.x = pose_est.getLinearVelocity();
    odom_msg.twist.twist.angular.z = gz_for_odom;
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
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
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
    rcl_timer_t timer_ctrl, timer_odom;
    RCCHECK(rclc_timer_init_default(&timer_ctrl, &support, RCL_MS_TO_NS(20), timer_callback_ctrl));
    RCCHECK(rclc_timer_init_default(&timer_odom, &support, RCL_MS_TO_NS(20), timer_callback_odom));
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    while (rmw_uros_ping_agent(100, 3) != RCL_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    rmw_uros_sync_session(1000);
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_ctrl));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_odom));
    init_odom_msg();
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
extern "C" void app_main(void) {
    setupPins();
    xTaskCreate(setupRos, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}
