#include "message_center.h"
#include "stdlib.h"
#include "string.h"
#include "bsp_log.h"
#include "stm32f4xx.h"

/* message_center是fake head node,是方便链表编写的技巧,这样就不需要处理链表头的特殊情况 */
static Publisher_t message_center = {
    .topic_name = "Message_Manager",
    .first_subs = NULL,
    .next_topic_node = NULL};

static uint8_t GetTopicCount(void)
{
    uint8_t topic_count = 0;
    Publisher_t *iter = message_center.next_topic_node;

    while (iter)
    {
        topic_count++;
        iter = iter->next_topic_node;
    }

    return topic_count;
}

static void DestroySubscriber(Subscriber_t *subscriber)
{
    if (subscriber == NULL)
    {
        return;
    }

    for (size_t i = 0; i < QUEUE_SIZE; ++i)
    {
        free(subscriber->queue[i]);
    }
    free(subscriber);
}

static Publisher_t *CreatePublisher(char *name, uint8_t data_len)
{
    if (GetTopicCount() >= MAX_TOPIC_COUNT)
    {
        LOGERROR("TOPIC COUNT FULL:%s", name);
        return NULL;
    }

    Publisher_t *new_topic = (Publisher_t *)malloc(sizeof(Publisher_t));
    if (new_topic == NULL)
    {
        LOGERROR("ALLOC TOPIC FAILED:%s", name);
        return NULL;
    }

    memset(new_topic, 0, sizeof(Publisher_t));
    new_topic->data_len = data_len;
    strcpy(new_topic->topic_name, name);
    return new_topic;
}

static Subscriber_t *CreateSubscriber(uint8_t data_len)
{
    Subscriber_t *new_subscriber = (Subscriber_t *)malloc(sizeof(Subscriber_t));
    if (new_subscriber == NULL)
    {
        LOGERROR("ALLOC SUBSCRIBER FAILED");
        return NULL;
    }

    memset(new_subscriber, 0, sizeof(Subscriber_t));
    new_subscriber->data_len = data_len;
    for (size_t i = 0; i < QUEUE_SIZE; ++i)
    {
        new_subscriber->queue[i] = malloc(data_len);
        if (new_subscriber->queue[i] == NULL)
        {
            LOGERROR("ALLOC SUBSCRIBER QUEUE FAILED:%d", (int)data_len);
            DestroySubscriber(new_subscriber);
            return NULL;
        }
    }

    return new_subscriber;
}

static inline uint32_t MessageCenterEnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void MessageCenterExitCritical(uint32_t primask)
{
    if ((primask & 0x1U) == 0U)
    {
        __enable_irq();
    }
}

static uint8_t CheckName(char *name)
{
    if (name == NULL)
    {
        LOGERROR("EVENT NAME NULL");
        return 0;
    }

    if (strnlen(name, MAX_TOPIC_NAME_LEN + 1) > MAX_TOPIC_NAME_LEN)
    {
        LOGERROR("EVENT NAME TOO LONG:%s", name);
        while (1)
            ; // 进入这里说明话题名超出长度限制
    }

    return 1;
}

static void CheckLen(uint8_t len1, uint8_t len2)
{
    if (len1 != len2)
    {
        LOGERROR("EVENT LEN NOT SAME:%d,%d", len1, len2);
        while (1)
            ; // 进入这里说明相同话题的消息长度却不同
    }
}

Publisher_t *RegisterPublisher(char *name, uint8_t data_len)
{
    if (CheckName(name) == 0U)
    {
        return NULL;
    }

    Publisher_t *node = &message_center;
    while (node->next_topic_node) // message_center会直接跳过,不需要特殊处理,这被称作dumb_head(编程技巧)
    {
        node = node->next_topic_node;            // 切换到下一个发布者(话题)结点
        if (strcmp(node->topic_name, name) == 0) // 如果已经注册了相同的话题,直接返回结点指针
        {
            CheckLen(data_len, node->data_len);
            return node;
        }
    } // 遍历完发现尚未创建name对应的话题
    // 在链表尾部创建新的话题并初始化
    node->next_topic_node = CreatePublisher(name, data_len);
    if (node->next_topic_node == NULL)
    {
        return NULL;
    }

    return node->next_topic_node;
}

Subscriber_t *RegisterSubscriber(char *name, uint8_t data_len)
{
    Publisher_t *pub = RegisterPublisher(name, data_len); // 查找或创建该话题的发布者
    if (pub == NULL)
    {
        return NULL;
    }

    // 创建新的订阅者结点,申请内存,注意要memset因为新空间不一定是空的,可能有之前留存的垃圾值
    Subscriber_t *ret = CreateSubscriber(data_len);
    if (ret == NULL)
    {
        return NULL;
    }

    // 如果是第一个订阅者,特殊处理一下,将first_subs指针指向新建的订阅者(详见文档)
    if (pub->first_subs == NULL)
    {
        pub->first_subs = ret;
        return ret;
    }
    // 若该话题已经有订阅者, 遍历订阅者链表,直到到达尾部
    Subscriber_t *sub = pub->first_subs; // 作为iterator
    while (sub->next_subs_queue)         // 遍历订阅了该话题的订阅者链表
    {
        sub = sub->next_subs_queue; // 移动到下一个订阅者,遇到空指针停下,说明到了链表尾部
    }
    sub->next_subs_queue = ret; // 把刚刚创建的订阅者接上
    return ret;
}

/* 如果队列为空,会返回0;成功获取数据,返回1;后续可以做更多的修改,比如剩余消息数目等 */
uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr)
{
    if (sub == NULL || data_ptr == NULL)
    {
        return 0;
    }

    uint32_t primask = MessageCenterEnterCritical();
    if (sub->temp_size == 0)
    {
        MessageCenterExitCritical(primask);
        return 0;
    }
    memcpy(data_ptr, sub->queue[sub->front_idx], sub->data_len);
    sub->front_idx = (sub->front_idx + 1U) % QUEUE_SIZE; // 队列头索引增加
    sub->temp_size--;                                 // pop一个数据,长度减1
    MessageCenterExitCritical(primask);
    return 1;
}

uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr)
{
    if (pub == NULL || data_ptr == NULL)
    {
        return 0;
    }

    uint8_t pushed_count = 0;
    Subscriber_t *iter = pub->first_subs; // iter作为订阅者指针,遍历订阅该话题的所有订阅者;如果为空说明遍历结束
    // 遍历订阅了当前话题的所有订阅者,依次填入最新消息
    while (iter)
    {
        uint32_t primask = MessageCenterEnterCritical();
        Subscriber_t *next = iter->next_subs_queue;
        if (iter->temp_size == QUEUE_SIZE) // 如果队列已满,则需要删除最老的数据(头部),再填入
        {
            // 队列头索引前移动,相当于抛弃前一个位置的数据,被抛弃的位置稍后会被写入新的数据
            iter->front_idx = (iter->front_idx + 1) % QUEUE_SIZE;
            iter->temp_size--; // 相当于出队,size-1
        }
        // 将Pub的数据复制到队列的尾部(最新)
        memcpy(iter->queue[iter->back_idx], data_ptr, pub->data_len);
        iter->back_idx = (iter->back_idx + 1) % QUEUE_SIZE; // 队列尾部前移
        iter->temp_size++;                                  // 入队,size+1
        MessageCenterExitCritical(primask);
        pushed_count++;

        iter = next; // 访问下一个订阅者
    }
    return pushed_count;
}
