# redisObject
robj是redis的统一访问接口，robj可以是各种数据类型voidz指针。
```c
typedef struct redisObject {
    unsigned type:4;
    unsigned encoding:4;
    unsigned lru:REDIS_LRU_BITS; /* lru time (relative to server.lruclock) */
    int refcount;
    void *ptr;
} robj;

/* Object types */
#define REDIS_STRING 0
#define REDIS_LIST 1
#define REDIS_SET 2
#define REDIS_ZSET 3
#define REDIS_HASH 4

/* Objects encoding. Some kind of objects like Strings and Hashes can be
 * internally represented in multiple ways. The 'encoding' field of the object
 * is set to one of this fields for this object. */
#define REDIS_ENCODING_RAW 0     /* Raw representation */
#define REDIS_ENCODING_INT 1     /* Encoded as integer */
#define REDIS_ENCODING_HT 2      /* Encoded as hash table */
#define REDIS_ENCODING_ZIPMAP 3  /* Encoded as zipmap */
#define REDIS_ENCODING_LINKEDLIST 4 /* Encoded as regular linked list */
#define REDIS_ENCODING_ZIPLIST 5 /* Encoded as ziplist */
#define REDIS_ENCODING_INTSET 6  /* Encoded as intset */
#define REDIS_ENCODING_SKIPLIST 7  /* Encoded as skiplist */
#define REDIS_ENCODING_EMBSTR 8  /* Embedded sds string encoding */
```
一个robj，type在定义时规定，对外表现的数据结构，encoding可以改变，为底层的具体实现数据结构。

# string
## sds， RawString
```c
typedef char *sds;  // 程序中sds和sdshdr可以相互转换，sds必须通过专用函数进行修改，这个类型别名实际隐藏了很多细节，必须理解其危险性
```
```c
struct sdshdr {
    int len;
    int free;
    char buf[];  // 该指针不占用任何空间，sdshdr为字符串的起始位置
};
```
二者可以使用以下方式转换
```c
const sds s;
struct sdshdr *sh = (void*)(s-(sizeof(struct sdshdr)));
```

sds实际上为c字符串的包装，在其前面加上其长度了剩余空间。是一个简单的，内存安全的string,

## embeddedString，
```c
robj *createEmbeddedStringObject(char *ptr, size_t len) {
    // 相比RawString， 只是obj和string内存绑定在一起了
    robj *o = zmalloc(sizeof(robj)+sizeof(struct sdshdr)+len+1);
    struct sdshdr *sh = (void*)(o+1);

    o->type = REDIS_STRING;
    o->encoding = REDIS_ENCODING_EMBSTR;
    o->ptr = sh+1;
    o->refcount = 1;
    o->lru = LRU_CLOCK();

    sh->len = len;
    sh->free = 0;
    if (ptr) {
        memcpy(sh->buf,ptr,len);
        sh->buf[len] = '\0';
    } else {
        memset(sh->buf,0,len+1);
    }
    return o;
}
```
robj+sdshdr+sds最大为64字节

##  REDIS_ENCODING_INT
```c
 o = createObject(REDIS_STRING, NULL);
 o->encoding = REDIS_ENCODING_INT;
 o->ptr = (void*)((long)value);
 ```

## list
### 基本定义
```c
typedef struct list {
    listNode *head;
    listNode *tail;
    void *(*dup)(void *ptr);
    void (*free)(void *ptr);
    int (*match)(void *ptr, void *key);
    unsigned long len;
} list;
// 节点
typedef struct listNode {
    struct listNode *prev;
    struct listNode *next;
    void *value;
} listNode;
// 迭代器
typedef struct listIter {
    listNode *next;
    int direction;
} listIter;
```
### LINKEDLIST， 默认编码
默认list,  list是独立对象
```c
robj *createListObject(void) {
    list *l = listCreate();
    robj *o = createObject(REDIS_LIST,l);
    // 将引用计数减1
    listSetFreeMethod(l,decrRefCountVoid);
    o->encoding = REDIS_ENCODING_LINKEDLIST;
    return o;
}
```

### ZIPLIST
ZIPLIST的数据占用一整块空间，当数据量增加的时候，插入数据效率会很差。
看mk文档
area        |<---- ziplist header ---->|<----------- entries ------------->|<-end->|

size          4 bytes  4 bytes  2 bytes    ?        ?        ?        ?     1 byte
            +---------+--------+-------+--------+--------+--------+--------+-------+
component   | zlbytes | zltail | zllen | entry1 | entry2 |  ...   | entryN | zlend |
            +---------+--------+-------+--------+--------+--------+--------+-------+
                                       ^                          ^        ^
address                                |                          |        |
                                ZIPLIST_ENTRY_HEAD                |   ZIPLIST_ENTRY_END
                                                                  |
                                                        ZIPLIST_ENTRY_TAIL

## SET
set的实现有两种，分别是dict 和 insetNew
### REDIS_ENCODING_HT, 底层实现是hashtable
### INTSET
实际是一个数组，根据数组元素进行编码

## RedisHash
### ziplist

## zset
### zset REDIS_ENCODING_SKIPLIST
```c
typedef struct zset {
    dict *dict;
    zskiplist *zsl;
} zset; 
```
### ziplist
zset也可以用ziplist实现
