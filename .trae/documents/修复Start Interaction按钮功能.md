1. 确保InteractionWindow在所有模式下都能正确创建
2. 修复主题绑定问题，确保在所有模式下都正确定义和绑定主题
3. 优化start_interaction方法，确保在debug模式下能正确停止
4. 测试修复后的功能

具体修改内容：
1. 在replay模式下添加PauseButtonTheme和ResumeButtonTheme的定义
2. 确保InteractionWindow在所有需要的模式下创建
3. 检查并修复主题绑定逻辑
4. 优化start_interaction方法的实现，确保调试时能正确触发