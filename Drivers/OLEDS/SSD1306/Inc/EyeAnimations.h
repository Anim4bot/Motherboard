#ifndef INC_EYEANIMATIONS_H_
#define INC_EYEANIMATIONS_H_


#define OLED_WIDTH  64
#define OLED_HEIGHT 48

typedef enum
{
	WakingUp,
	GoingToSleep,
	Sleeping,
	Sleepy,
	Follow,
	Neutral,
	Blink,
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


typedef struct
{
	Expressions_enum Expression;
	uint8_t Contrast;
}
Eyes_st;
Eyes_st Eyes;


#endif /* INC_EYEANIMATIONS_H_ */
