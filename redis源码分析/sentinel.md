# sentinel
哨兵也是redis服务器，与普通服务器相比，其服务的命令不同。但事件模型相同。
## 相关数据结构介绍
### sentinelRedisInstance
类型 SRI_MASTER|SRI_SLAVE|SRI_SENTINEL
```c
// Sentinel 会为每个被监视的 Redis 实例创建相应的 sentinelRedisInstance 实例
// （被监视的实例可以是主服务器、从服务器、或者其他 Sentinel ）
typedef struct sentinelRedisInstance {
    
    // 标识值，记录了实例的类型，以及该实例的当前状态
    int flags;      /* See SRI_... defines */
    
    // 实例的名字
    // 主服务器的名字由用户在配置文件中设置
    // 从服务器以及 Sentinel 的名字由 Sentinel 自动设置
    // 格式为 ip:port ，例如 "127.0.0.1:26379"
    char *name;     /* Master name from the point of view of this sentinel. */

    // 实例的运行 ID
    char *runid;    /* run ID of this instance. */

    // 配置纪元，用于实现故障转移
    uint64_t config_epoch;  /* Configuration epoch. */

    // 实例的地址
    sentinelAddr *addr; /* Master host. */

    // 用于发送命令的异步连接
    redisAsyncContext *cc; /* Hiredis context for commands. */

    // 用于执行 SUBSCRIBE 命令、接收频道信息的异步连接
    // 仅在实例为主服务器时使用
    redisAsyncContext *pc; /* Hiredis context for Pub / Sub. */

    // 已发送但尚未回复的命令数量
    int pending_commands;   /* Number of commands sent waiting for a reply. */

    // cc 连接的创建时间
    mstime_t cc_conn_time; /* cc connection time. */
    
    // pc 连接的创建时间
    mstime_t pc_conn_time; /* pc connection time. */

    // 最后一次从这个实例接收信息的时间
    mstime_t pc_last_activity; /* Last time we received any message. */

    // 实例最后一次返回正确的 PING 命令回复的时间
    mstime_t last_avail_time; /* Last time the instance replied to ping with
                                 a reply we consider valid. */
    // 实例最后一次发送 PING 命令的时间
    mstime_t last_ping_time;  /* Last time a pending ping was sent in the
                                 context of the current command connection
                                 with the instance. 0 if still not sent or
                                 if pong already received. */
    // 实例最后一次返回 PING 命令的时间，无论内容正确与否
    mstime_t last_pong_time;  /* Last time the instance replied to ping,
                                 whatever the reply was. That's used to check
                                 if the link is idle and must be reconnected. */

    // 最后一次向频道发送问候信息的时间
    // 只在当前实例为 sentinel 时使用
    mstime_t last_pub_time;   /* Last time we sent hello via Pub/Sub. */

    // 最后一次接收到这个 sentinel 发来的问候信息的时间
    // 只在当前实例为 sentinel 时使用
    mstime_t last_hello_time; /* Only used if SRI_SENTINEL is set. Last time
                                 we received a hello from this Sentinel
                                 via Pub/Sub. */

    // 最后一次回复 SENTINEL is-master-down-by-addr 命令的时间
    // 只在当前实例为 sentinel 时使用
    mstime_t last_master_down_reply_time; /* Time of last reply to
                                             SENTINEL is-master-down command. */

    // 实例被判断为 SDOWN 状态的时间
    mstime_t s_down_since_time; /* Subjectively down since time. */

    // 实例被判断为 ODOWN 状态的时间
    mstime_t o_down_since_time; /* Objectively down since time. */

    // SENTINEL down-after-milliseconds 选项所设定的值
    // 实例无响应多少毫秒之后才会被判断为主观下线（subjectively down）
    mstime_t down_after_period; /* Consider it down after that period. */

    // 从实例获取 INFO 命令的回复的时间
    mstime_t info_refresh;  /* Time at which we received INFO output from it. */

    /* Role and the first time we observed it.
     * This is useful in order to delay replacing what the instance reports
     * with our own configuration. We need to always wait some time in order
     * to give a chance to the leader to report the new configuration before
     * we do silly things. */
    // 实例的角色
    int role_reported;
    // 角色的更新时间
    mstime_t role_reported_time;

    // 最后一次从服务器的主服务器地址变更的时间
    mstime_t slave_conf_change_time; /* Last time slave master addr changed. */

    /* Master specific. */
    /* 主服务器实例特有的属性 -------------------------------------------------------------*/

    // 其他同样监控这个主服务器的所有 sentinel
    dict *sentinels;    /* Other sentinels monitoring the same master. */

    // 如果这个实例代表的是一个主服务器
    // 那么这个字典保存着主服务器属下的从服务器
    // 字典的键是从服务器的名字，字典的值是从服务器对应的 sentinelRedisInstance 结构
    dict *slaves;       /* Slaves for this master instance. */

    // SENTINEL monitor <master-name> <IP> <port> <quorum> 选项中的 quorum 参数
    // 判断这个实例为客观下线（objectively down）所需的支持投票数量
    int quorum;         /* Number of sentinels that need to agree on failure. */

    // SENTINEL parallel-syncs <master-name> <number> 选项的值
    // 在执行故障转移操作时，可以同时对新的主服务器进行同步的从服务器数量
    int parallel_syncs; /* How many slaves to reconfigure at same time. */

    // 连接主服务器和从服务器所需的密码
    char *auth_pass;    /* Password to use for AUTH against master & slaves. */

    /* Slave specific. */
    /* 从服务器实例特有的属性 -------------------------------------------------------------*/

    // 主从服务器连接断开的时间
    mstime_t master_link_down_time; /* Slave replication link down time. */

    // 从服务器优先级
    int slave_priority; /* Slave priority according to its INFO output. */

    // 执行故障转移操作时，从服务器发送 SLAVEOF <new-master> 命令的时间
    mstime_t slave_reconf_sent_time; /* Time at which we sent SLAVE OF <new> */

    // 主服务器的实例（在本实例为从服务器时使用）
    struct sentinelRedisInstance *master; /* Master instance if it's slave. */

    // INFO 命令的回复中记录的主服务器 IP
    char *slave_master_host;    /* Master host as reported by INFO */
    
    // INFO 命令的回复中记录的主服务器端口号
    int slave_master_port;      /* Master port as reported by INFO */

    // INFO 命令的回复中记录的主从服务器连接状态
    int slave_master_link_status; /* Master link status as reported by INFO */

    // 从服务器的复制偏移量
    unsigned long long slave_repl_offset; /* Slave replication offset. */

    /* Failover */
    /* 故障转移相关属性 -------------------------------------------------------------------*/


    // 如果这是一个主服务器实例，那么 leader 将是负责进行故障转移的 Sentinel 的运行 ID 。
    // 如果这是一个 Sentinel 实例，那么 leader 就是被选举出来的领头 Sentinel 。
    // 这个域只在 Sentinel 实例的 flags 属性的 SRI_MASTER_DOWN 标志处于打开状态时才有效。
    char *leader;       /* If this is a master instance, this is the runid of
                           the Sentinel that should perform the failover. If
                           this is a Sentinel, this is the runid of the Sentinel
                           that this Sentinel voted as leader. */
    // 领头的纪元
    uint64_t leader_epoch; /* Epoch of the 'leader' field. */
    // 当前执行中的故障转移的纪元
    uint64_t failover_epoch; /* Epoch of the currently started failover. */
    // 故障转移操作的当前状态
    int failover_state; /* See SENTINEL_FAILOVER_STATE_* defines. */

    // 状态改变的时间
    mstime_t failover_state_change_time;

    // 最后一次进行故障迁移的时间
    mstime_t failover_start_time;   /* Last failover attempt start time. */

    // SENTINEL failover-timeout <master-name> <ms> 选项的值
    // 刷新故障迁移状态的最大时限
    mstime_t failover_timeout;      /* Max time to refresh failover state. */

    mstime_t failover_delay_logged; /* For what failover_start_time value we
                                       logged the failover delay. */
    // 指向被提升为新主服务器的从服务器的指针
    struct sentinelRedisInstance *promoted_slave; /* Promoted slave instance. */

    /* Scripts executed to notify admin or reconfigure clients: when they
     * are set to NULL no script is executed. */
    // 一个文件路径，保存着 WARNING 级别的事件发生时执行的，
    // 用于通知管理员的脚本的地址
    char *notification_script;

    // 一个文件路径，保存着故障转移执行之前、之后、或者被中止时，
    // 需要执行的脚本的地址
    char *client_reconfig_script;

} sentinelRedisInstance;```
## sentinel的工作方式
### 初始化

1. 检查服务器是否以Sentinel启动
```c
int checkForSentinelMode(int argc, char **argv) {
    int j;

    if (strstr(argv[0],"redis-sentinel") != NULL) return 1;
    for (j = 1; j < argc; j++)
        if (!strcmp(argv[j],"--sentinel")) return 1;
    return 0;
}
```
返回为1，则此进程为sentine进程
2. 初始化sential配置
```c
// main()函数中
if (server.sentinel_mode) {
    initSentinelConfig(); // server.port = REDIS_SENTINEL_PORT
    initSentinel();
}
void initSentinel(void) {
    int j;

    /* Remove usual Redis commands from the command table, then just add
     * the SENTINEL command. */

    // 清空 Redis 服务器的命令表（该表用于普通模式）
    dictEmpty(server.commands,NULL);
    // 将 SENTINEL 模式所用的命令添加进命令表
    for (j = 0; j < sizeof(sentinelcmds)/sizeof(sentinelcmds[0]); j++) {
        int retval;
        struct redisCommand *cmd = sentinelcmds+j;

        retval = dictAdd(server.commands, sdsnew(cmd->name), cmd);
        redisAssert(retval == DICT_OK);
    }

    /* Initialize various data structures. */
    /* 初始化 Sentinel 的状态 */
    // 初始化纪元
    sentinel.current_epoch = 0;

    // 初始化保存主服务器信息的字典
    sentinel.masters = dictCreate(&instancesDictType,NULL);

    // 初始化 TILT 模式的相关选项
    sentinel.tilt = 0;
    sentinel.tilt_start_time = 0;
    sentinel.previous_time = mstime();

    // 初始化脚本相关选项
    sentinel.running_scripts = 0;
    sentinel.scripts_queue = listCreate();
}
```
3. 从配置文件中读取master相关信息
- 不同的配置文件在不同行
- 将配置文件和option汇总为config, 后者在实际执行中会覆盖前者
- 分别配置各个关键字，以sentinel开头的关键字表示是sentinel特有配置
```c
if (!strcasecmp(argv[0],"sentinel")) {
/* argc == 1 is handled by main() as we need to enter the sentinel
    * mode ASAP. */
// 如果 SENTINEL 命令不为空，那么执行以下代码
if (argc != 1) {
    // 如果 SENTINEL 模式未开启，那么出错
    if (!server.sentinel_mode) {
        err = "sentinel directive while not in sentinel mode";
        goto loaderr;
    }
    // 载入 SENTINEL 相关选项
    err = sentinelHandleConfiguration(argv+1,argc-1);
    if (err) goto loaderr;
}
```
sentinelHandleConfiguration会获取需要监视的master对象，并设置监视策略。***slave的型信息不会从此处获取***
- 创建主服务器实例，使用配置信息（createSentinelRedisInstance）
### 初始化运行 sentinelIsRunning()
```c
void sentinelIsRunning(void) {
    redisLog(REDIS_WARNING,"Sentinel runid is %s", server.runid);

    // Sentinel 不能在没有配置文件的情况下执行
    if (server.configfile == NULL) {
        redisLog(REDIS_WARNING,
            "Sentinel started without a config file. Exiting...");
        exit(1);
    } else if (access(server.configfile,W_OK) == -1) {
        redisLog(REDIS_WARNING,
            "Sentinel config file %s is not writable: %s. Exiting...",
            server.configfile,strerror(errno));
        exit(1);
    }

    /* We want to generate a +monitor event for every configured master
     * at startup. */
    sentinelGenerateInitialMonitorEvents(); 
}```

### 定时执行sentinelTimer
```c
// sentinel 模式的主函数，由 redis.c/serverCron 函数调用
void sentinelTimer(void) {

    // 记录本次 sentinel 调用的事件，
    // 并判断是否需要进入 TITL 模式
    sentinelCheckTiltCondition();

    // 执行定期操作
    // 比如 PING 实例、分析主服务器和从服务器的 INFO 命令
    // 向其他监视相同主服务器的 sentinel 发送问候信息
    // 并接收其他 sentinel 发来的问候信息
    // 执行故障转移操作，等等
    sentinelHandleDictOfRedisInstances(sentinel.masters);

    // 运行等待执行的脚本
    sentinelRunPendingScripts();

    // 清理已执行完毕的脚本，并重试出错的脚本
    sentinelCollectTerminatedScripts();

    // 杀死运行超时的脚本
    sentinelKillTimedoutScripts();

    /* We continuously change the frequency of the Redis "timer interrupt"
     * in order to desynchronize every Sentinel from every other.
     * This non-determinism avoids that Sentinels started at the same time
     * exactly continue to stay synchronized asking to be voted at the
     * same time again and again (resulting in nobody likely winning the
     * election because of split brain voting). */
    server.hz = REDIS_DEFAULT_HZ + rand() % REDIS_DEFAULT_HZ;
}

```
## sentinel原理