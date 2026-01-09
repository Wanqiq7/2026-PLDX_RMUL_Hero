#include "VOFA.h"

#include <string.h>

// JustFloat 帧尾：0x00 0x00 0x80 0x7f
static const uint8_t k_justfloat_tail[4] = {0x00, 0x00, 0x80, 0x7F};

uint16_t VofaGetJustFloatFrameLen(uint8_t channels) {
  return (uint16_t)((uint16_t)channels * (uint16_t)sizeof(float) +
                    (uint16_t)sizeof(k_justfloat_tail));
}

uint8_t VofaJustFloatSend(const VofaJustFloatSender_s *sender,
                          const float *values, uint8_t channels) {
  if (sender == NULL || values == NULL || channels == 0U) {
    return 0U;
  }

  if (sender->ctx == NULL || sender->is_ready == NULL || sender->send == NULL) {
    return 0U;
  }

  if (sender->tx_buf == NULL || sender->tx_buf_len == 0U) {
    return 0U;
  }

  if (sender->max_channels == 0U || channels > sender->max_channels) {
    return 0U;
  }

  uint16_t frame_len = VofaGetJustFloatFrameLen(channels);
  if (frame_len > sender->tx_buf_len) {
    return 0U;
  }

  // 传输层约束：若底层异步发送未完成，重复发送可能取消上一次
  if (!sender->is_ready(sender->ctx)) {
    return 0U;
  }

  uint16_t payload_len = (uint16_t)channels * (uint16_t)sizeof(float);

  // STM32F4 为小端，float 内存布局可直接 memcpy；JustFloat 以小端 float32 传输
  memcpy(sender->tx_buf, (const void *)values, payload_len);
  memcpy(&sender->tx_buf[payload_len], k_justfloat_tail,
         sizeof(k_justfloat_tail));

  sender->send(sender->ctx, sender->tx_buf, frame_len, sender->transfer_mode);
  return 1U;
}
