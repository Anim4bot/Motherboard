#ifndef INC_EYEANIMATIONS_H_
#define INC_EYEANIMATIONS_H_


typedef enum
{
	WakingUp,
	GoingToSleep,
	Sleeping,
	Sleepy,
	Neutral,
	Blink,
	BlinkHigh,
	BlinkLow,
	BlinkLeft,
	BlinkRight,
	Happy,
	Sad,
	Worried,
	Focused,
	Annoyed,
	Surprised,
	Skeptic,
	Squint,
	Frustrated,
	Unimpressed,
	Angry,
	Furious,
	Scared,
	Awe
}
Expressions_enum;


typedef enum
{
	fast     = 1,
	medium   = 2,
	slow     = 3,
	verySlow = 5
}
AnimSpeed_enum;

typedef struct
{
	Expressions_enum Expression;
	uint8_t Contrast;
}
Eyes_st;
Eyes_st Eyes;


#endif /* INC_EYEANIMATIONS_H_ */
