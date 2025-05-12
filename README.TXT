2022-03-11
1.修改ADC3配置错误的问题导致泵控相电压采集错误，无法检测泵控电机开路的问题。
2.增加驱动电机开路检测功能，上电后V相输出50%，U,W全关，检测U和W是否有电压。
3.上电后关闭U,W,P输出使能，自检完毕后在PwmEnablePwOn中打开输出使能。
4.增加了28#电机开路检测报警。

Hardware V2.0
1.修改泵控为双桥臂控制，单桥控制没有续流回路导致控制器发热严重。
2.PA0的POT2_HIGH改为DRIVER3_R
  泵控的TIM5改为TIM20,原来的TIM5_CH3改为DRIVER5_R 
  TIM20 CH1,CH1N改为PE2,PE4.	

Hardware V1.0
1.需要修改选项字否则Boot0和CAN接口冲突，nSWBOOT0=0; nBOOT0=1;
