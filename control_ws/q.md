5.18 下午 yu

更改：
1. 更改取点顺序，改成cab


后续可改善：
1. g_last_result_task_id 这个变量是否有些多余？
2. 您目前的架构是用Topic配合全局变量来实现请求-应答。是否可以使用原生的 ROS Service 和 ROS Action呢？
3. movetoPoint函数是否可以进行改进，使用原生的ros提供的方法 SimpleActionClient下的waitForResult()。 另一个是ACTIVE是否导致死循环



a点任务点不太准

