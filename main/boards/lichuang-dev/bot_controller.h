#ifndef _BOT_CONTROLLER_H_
#define _BOT_CONTROLLER_H_

// 舵机ID宏定义
#define SERVO_LEFT_ARM_ID      2    // 左臂舵机
#define SERVO_RIGHT_ARM_ID     3    // 右臂舵机
#define SERVO_LEFT_EYEBROW_ID  4    // 左眉舵机
#define SERVO_RIGHT_EYEBROW_ID 5    // 右眉舵机
#define SERVO_LEFT_EYE_ID      6    // 左眼角舵机
#define SERVO_RIGHT_EYE_ID     7    // 右眼角舵机
#define SERVO_HEAD_TURN_ID     8    // 头部左右转动舵机
#define SERVO_NECK_UPPER_ID    9    // 脖子大臂上下旋转舵机
#define SERVO_NECK_LOWER_ID    10   // 脖子小臂上下旋转舵机

class BotController {
   private:
    static BotController* instance_;

    // 私有构造函数
    BotController();
    void RegisterMcpTools();

    // 舵机控制私有方法
    bool SetLeftArmAngle(int angle);
    bool SetRightArmAngle(int angle);
    bool SetLeftEyebrowAngle(int angle);
    bool SetRightEyebrowAngle(int angle);
    bool SetLeftEyeAngle(int angle);
    bool SetRightEyeAngle(int angle);
    bool SetHeadTurnAngle(int angle);
    bool SetNeckUpperAngle(int angle);
    bool SetNeckLowerAngle(int angle);

   public:
    static BotController* GetInstance();
    ~BotController();
};

#endif  // _BOT_CONTROLLER_H_