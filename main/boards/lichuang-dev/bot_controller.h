#ifndef _BOT_CONTROLLER_H_
#define _BOT_CONTROLLER_H_

class BotController {
   private:
    static BotController* instance_;

    // 私有构造函数
    BotController();
    void RegisterMcpTools();

   public:
    static BotController* GetInstance();
    ~BotController();
};

#endif  // _BOT_CONTROLLER_H_