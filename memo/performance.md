以下のコードを実行
```
crate::led::on(crate::led::LedColor::Red)?;
let volt_r = calc_ff_r(ctx, target_v);
let volt_l = calc_ff_l(ctx, target_v);
let duty_r = calc_duty(ctx, volt_r);
let duty_l = calc_duty(ctx, volt_l);
crate::led::off(crate::led::LedColor::Red)?;
```

`#[inline(always)]`あり40-50us
`#[inline(always)]`なし50-60us

update 90 -110

mesure 800-1000over


let batt = wall_sensor::read_batt()?; 110-130
unwrapを減らした 80-200

measure release 


SEN_LS GPIO1 ADC1_CH0
SEN_LF GPIO2 ADC1_CH1
SEN_RF GPIO3 ADC1_CH2
SEN_RS GPIO4 ADC1_CH3
BT_SEN GPIO5 ADC1_CH4

Elapsed: 156 ms
calc_volt_duty: 60 20 us
set_duty: 123 25 us
call_measure: 818 88 us
call_update: 653 65 us
call_log: 599 73 us
