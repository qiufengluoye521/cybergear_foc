ECAT_StateChange
    AL_ControlInd

COE的状态机图如下所示
+---------------------------------------------------------------------------------+
|                                 POWER ON / RESET                                |
+---------------------------------------------------------------------------------+
                                         |
                                         v
+---------------------------------------------------------------------------------+
| ESM: Init                 |  CoE: Initialization                                |
+---------------------------------------------------------------------------------+
                                         |
                                         | (主站配置邮箱通信 / 自动初始化完成)
                                         v
+---------------------------------------------------------------------------------+
| ESM: Pre-Operational      |  CoE: Pre-Operational                               |
|                           |  - 允许 SDO 读写对象字典 (OD)                       |
|                           |  - 禁止 PDO 过程数据传输                            |
+---------------------------------------------------------------------------------+
                                         |
                                         | (主站配置 PDO 映射并请求切换)
                                         v
+---------------------------------------------------------------------------------+
| ESM: Safe-Operational     |  CoE: Pre-Operational                               |
|                           |  - 允许 SDO 通信                                    |
|                           |  - 仅允许 输入PDO (Input/TxPDO) 刷新                |
+---------------------------------------------------------------------------------+
                                         |
                      +------------------+------------------+
                      | (Start Remote Node 0x01)            | (Stop Node 0x02)
                      v                                     v
+-----------------------------------------+   +-----------------------------------+
| ESM: Operational    | CoE: Operational  |   | ESM: Safe-Op/Op | CoE: Stopped    |
| - 允许 SDO 通信                         |   | - 停止所有 SDO 和 PDO 通信        |
| - 允许 双向PDO (Input/Output) 周期刷新  |   | - 仅响应 NMT 基础网络管理命令     |
| - 电机驱动(CiA402)在此状态下使能工作    |   |                                   |
+-----------------------------------------+   +-----------------------------------+
                      |                                     |
                      +------------------<------------------+
                                  (Enter Pre-Op 0x80)



```mermaid
stateDiagram-v2
    [*] --> Init_State : Power On / Reset
    
    state "1. Init 状态 (底层准备)" as Init_State {
        ESM_Init --> CoE_Init : 硬件自检与参数初始化
    }

    Init_State --> PreOp_State : 主站配置 Mailbox (邮箱通信)
    
    state "2. Pre-Op 状态 (配置阶段)" as PreOp_State {
        direction <br>
        [*] --> CoE_PreOp
        note right of CoE_PreOp
            【SDO 交互通道开启】
            - 主站通过 SDO 配置对象字典
            - 映射 PDO 传输关系
            - 此时禁止任何 PDO 刷新
        end note
    }

    PreOp_State --> SafeOp_State : 主站启动状态检查
    
    state "3. Safe-Op 状态 (安全验证)" as SafeOp_State {
        [*] --> CoE_PreOp_Locked
        note right of CoE_PreOp_Locked
            【输入使能，输出锁定】
            - 允许主站读取从站数据 (Input PDO)
            - 禁止主站控制从站 (Output PDO 锁定)
            - 确保安全，防止设备误动作
        end note
    }

    SafeOp_State --> Op_State : 主站发送 NMT Start Node (0x01)
    SafeOp_State --> Stopped_State : 主站发送 NMT Stop Node (0x02)

    state "4. Operational 状态 (全功能运行)" as Op_State {
        ESM_Op --> CoE_Op
        note right of CoE_Op
            【生产运行状态】
            - SDO 与 双向PDO (Input/Output) 正常周期通信
            - 此时应用层 (如 CiA402 电机控制) 状态机开始工作
        end note
    }

    state "5. Stopped 状态 (安全停止)" as Stopped_State {
        ESM_Any --> CoE_Stopped
        note right of CoE_Stopped
            【应用层断开】
            - 禁止 SDO 和 PDO 通信
            - 节点仅响应 NMT 复位/启动指令
        end note
    }

    %% 返回与复位路径
    Op_State --> SafeOp_State : 链路降级 / 停止输出
    Op_State --> PreOp_State : 主站发送 NMT Enter Pre-Op (0x80)
    Stopped_State --> PreOp_State : 主站发送 NMT Enter Pre-Op (0x80)
    
    Op_State --> Init_State : NMT Reset Node (0x81)
    PreOp_State --> Init_State : NMT Reset Node (0x81)
    Stopped_State --> Init_State : NMT Reset Node (0x81)
```



状态                 AL 控制码          核心定位                CoE通信能力
INIT（初始化）	       0x01    硬件基础通信就绪，无上层配置	   无邮箱 / PDO 通信
PREOP（预运行）        0x02	   CoE 配置阶段，SDO 参数读写     仅 SM0/SM1 邮箱 (SDO/CoE)，无周期 PDO
SAFEOP（安全运行）     0x08     周期 PDO 校验，输出安全锁定    邮箱 + 周期输入 PDO 有效，输出强制安全值
OP （运行）            0x10     完整实时控制，全功能 PDO	  邮箱 + 输入 / 输出 PDO 全部正常交互
Bootstrap（引导，可选）	0x04	FoE 固件升级专用	          仅 FoE 文件下载通信
