#include "adapter/StudioBridge.h"

namespace fockit {

// 单实例静态指针（Commander 回调为裸函数指针）
StudioBridge* StudioBridge::self_ = nullptr;

} // namespace fockit
