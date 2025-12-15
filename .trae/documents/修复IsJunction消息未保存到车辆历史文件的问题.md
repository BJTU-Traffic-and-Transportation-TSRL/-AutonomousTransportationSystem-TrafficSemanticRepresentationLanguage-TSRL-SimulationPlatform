## 问题分析

通过分析代码和日志，我发现了以下问题：

1. 在`VehicleCommunicator.receive_message`方法中，当车辆接收广播消息时，创建的回复消息的`sender_id`被设置为`message.receiver_id`（即"broadcast"），这可能导致消息处理异常。

2. 虽然消息被正确接收并显示在`display_text.txt`中，但未被正确保存到车辆的消息历史文件中。

## 修复方案

1. 修改`VehicleCommunicator.receive_message`方法，确保当接收广播消息时，回复消息的`sender_id`使用车辆自身的ID，而不是"broadcast"。

2. 优化消息保存流程，确保所有接收的消息都能被正确保存到消息历史文件中。

3. 添加调试日志，便于跟踪消息处理流程。

## 修复步骤

1. 编辑`TSRL_interaction/communicator_category.py`文件，修改`VehicleCommunicator.receive_message`方法。
2. 确保回复消息的`sender_id`使用车辆自身的ID。
3. 优化消息保存逻辑，确保所有接收的消息都能被正确保存。
4. 添加调试日志，便于跟踪消息处理流程。
5. 测试修复效果，确保IsJunction消息能被正确保存到车辆历史文件中。

