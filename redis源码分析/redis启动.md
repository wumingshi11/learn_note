# redis启动流程
1. 设置和进程相关的数据，随机数，locale.
2. 判断是否为哨兵模式
3. 设置默认配置，initServerConfig(), 各个配置的具体含义待后续分析功能时查看
```c
    // 初始化并设置RDB策略
    resetServerSaveParams();
    appendServerSaveParams(60*60,1);  /* save after 1 hour and 1 change */
    appendServerSaveParams(300,100);  /* save after 5 minutes and 100 changes */
    appendServerSaveParams(60,10000); /* save after 1 minute and 10000 changes */
```
4. 哨兵初始化。
5. 检查用户指定配置，可以为配置文件或选项
6. 创建服务器并初始化服务(见下节)
7. 创建pid文件
8. 从AOF或者RDB中读取数据
9. 运行事件处理器到服务器关闭

## 创建服务器并初始化服务 initServer()
### 1.设置进程信号处理函数
```c
signal(SIGHUP, SIG_IGN);
signal(SIGPIPE, SIG_IGN); // 忽略因向一个已经关闭的网络端口发送数据产生的信号
setupSignalHandlers();
```
### 2.创建与其他机器相关数据结构
```c
    server.current_client = NULL;
    server.clients = listCreate();
    server.clients_to_close = listCreate();
    server.slaves = listCreate();
    server.monitors = listCreate();
    server.slaveseldb = -1; /* Force to emit the first SELECT command. */
    server.unblocked_clients = listCreate();
    server.ready_keys = listCreate();
    server.clients_waiting_acks = listCreate();
    server.get_ack_from_slaves = 0;
    server.clients_paused = 0;
```
### 3. 打开网络接口
```C
adjustOpenFilesLimit();
server.el = aeCreateEventLoop(server.maxclients+REDIS_EVENTLOOP_FDSET_INCR);
```
### 4. 创建数据库
```C
    for (j = 0; j < server.dbnum; j++) {
        server.db[j].dict = dictCreate(&dbDictType,NULL);
        server.db[j].expires = dictCreate(&keyptrDictType,NULL);
        server.db[j].blocking_keys = dictCreate(&keylistDictType,NULL);
        server.db[j].ready_keys = dictCreate(&setDictType,NULL);
        server.db[j].watched_keys = dictCreate(&keylistDictType,NULL);
        server.db[j].eviction_pool = evictionPoolAlloc();
        server.db[j].id = j;
        server.db[j].avg_ttl = 0;
    }
```

### 5. 创建PUBSUB相关结构
```c
    // 创建 PUBSUB 相关结构
    server.pubsub_channels = dictCreate(&keylistDictType,NULL);
    server.pubsub_patterns = listCreate();
    listSetFreeMethod(server.pubsub_patterns,freePubsubPattern);
    listSetMatchMethod(server.pubsub_patterns,listMatchPubsubPattern);
```

### 6. 创建时间事件serverCron，1ms执行一次
1. 主动清除过期键
2. 更新软件信息
3. 更新统计信息
4. 对数据库进行rehash
5. 触发BGSAVE和rewrite, 并处理之前的子进程
6. 客户端超时处理
7. 复制重连？？？
8. 更新看门狗
