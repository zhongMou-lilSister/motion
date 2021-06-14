#include "Control.h"

Control::MControl(int pin_lift, int pin_left, int pin_right)
{ 
	if (!servo_lift.attached()) servo_lift.attach(pin_lift);  //  lifting servo
	if (!servo_left.attached()) servo_left.attach(pin_left);  //  left servo
	if (!servo_right.attached()) servo_right.attach(pin_right);  //  right servo
	Lift(LIFT_STATE::LIFTED); delay(50);
  GoTo(rubberx, rubbery); delay(50);
}

void Control::DetachServo()
{
  servo_lift.detach();
  servo_left.detach();
  servo_right.detach();
}

void Control::Calibrate()
{
	while (true)
	{
//		GoTo(6.1, 30.7);
//		delay(1000);
//		GoTo(83.3, 26.7);
//		delay(1000);
    Lift(LIFT_STATE::LIFTED); delay(50);
    delay(500);
    Lift(LIFT_STATE::CLEAR); delay(50);
	}
}

void Control::Clear()
{
  DrawNumber(3, 3, 111, 1);
}

void Control::GoHome()
{
  Lift(LIFT_STATE::LIFTED);
  GoTo(rubberx, rubbery);
  Lift(LIFT_STATE::BETWEEN_NUMS);
  Serial.println(F("Ok, homed"));
  delay(50);
}


void Control::DrawNumber(float bx, float by, int num, float scale)
{
	switch (num)
	{
	case 0:
    GoTo(bx + 12 * scale, by + 6 * scale);
    Lift(LIFT_STATE::ON_DRAW);
    circleAntiClockwise(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
    delay(100);
    Lift(LIFT_STATE::BETWEEN_NUMS);
		break;
	case 1:
		GoTo(bx + 3 * scale, by + 15 * scale);
		Lift(LIFT_STATE::ON_DRAW);
		GoTo(bx + 10 * scale, by + 20 * scale);
		GoTo(bx + 10 * scale, by + 0 * scale);
    delay(100);
		Lift(LIFT_STATE::BETWEEN_NUMS);
		break;
	case 2:
		GoTo(bx + 2 * scale, by + 12 * scale);
		Lift(LIFT_STATE::ON_DRAW);
		circleClockwise(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -1.6, 1);
		GoTo(bx + 1 * scale, by + 0 * scale);
		GoTo(bx + 12 * scale, by + 0 * scale);
    delay(100);
		Lift(LIFT_STATE::BETWEEN_NUMS);
		break;
	case 3:
		GoTo(bx + 2 * scale, by + 17 * scale);
		Lift(LIFT_STATE::ON_DRAW);
		circleClockwise(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);
		circleClockwise(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
    delay(100);
		Lift(LIFT_STATE::BETWEEN_NUMS);
		break;
	case 4:
		GoTo(bx + 10 * scale, by + 0 * scale);
		Lift(LIFT_STATE::ON_DRAW);
		GoTo(bx + 10 * scale, by + 20 * scale);
		GoTo(bx + 2 * scale, by + 6 * scale);
		GoTo(bx + 12 * scale, by + 6 * scale);
    delay(100);
		Lift(LIFT_STATE::BETWEEN_NUMS);
		break;
	case 5:
		GoTo(bx + 2 * scale, by + 5 * scale);
		Lift(LIFT_STATE::ON_DRAW);
		circleAntiClockwise(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
		GoTo(bx + 5 * scale, by + 20 * scale);
		GoTo(bx + 12 * scale, by + 20 * scale);
    delay(100);
		Lift(LIFT_STATE::BETWEEN_NUMS);
		break;
	case 6:
		GoTo(bx + 2 * scale, by + 10 * scale);
		Lift(LIFT_STATE::ON_DRAW);
		circleClockwise(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
		GoTo(bx + 11 * scale, by + 20 * scale);
    delay(100);
		Lift(LIFT_STATE::BETWEEN_NUMS);
		break;
	case 7:
		GoTo(bx + 2 * scale, by + 20 * scale);
		Lift(LIFT_STATE::ON_DRAW);
		GoTo(bx + 12 * scale, by + 20 * scale);
		GoTo(bx + 2 * scale, by + 0);
    delay(100);
		Lift(LIFT_STATE::BETWEEN_NUMS);
		break;
	case 8:
		GoTo(bx + 5 * scale, by + 10 * scale);
		Lift(LIFT_STATE::ON_DRAW);
		circleClockwise(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);
		circleAntiClockwise(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
    delay(100);
		Lift(LIFT_STATE::BETWEEN_NUMS);
		break;

	case 9:
		GoTo(bx + 9 * scale, by + 11 * scale);
		Lift(LIFT_STATE::ON_DRAW);
		circleClockwise(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
		GoTo(bx + 5 * scale, by + 0);
    delay(100);
		Lift(LIFT_STATE::BETWEEN_NUMS);
		break;

	case 111:
		GoTo(rubberx, rubbery);
    Lift(LIFT_STATE::ON_DRAW); delay(100);
		GoTo(70, rubbery); delay(50);
		GoTo(65, 45); delay(50);

		GoTo(5, 45); delay(50);
		GoTo(5, 41); delay(50);
		GoTo(65, 41); delay(50);
		GoTo(65, 37); delay(50);

		GoTo(5, 37); delay(50);
		GoTo(5, 33); delay(50);
		GoTo(65, 33); delay(50);
		GoTo(65, 29); delay(50);

		GoTo(5, 29); delay(50);
		GoTo(5, 25); delay(50);
		GoTo(65, 25); delay(50);
		GoTo(65, 20); delay(50);

		GoTo(7, 20); delay(50);
		GoTo(7, rubbery); delay(50);

		GoTo(58, rubbery); delay(50);
    Lift(LIFT_STATE::CLEAR); delay(100);
		GoTo(rubberx, rubbery-1); delay(50);
		delay(200);
		Lift(LIFT_STATE::LIFTED);
    delay(300);
		break;

	case 11:
		GoTo(bx + 5 * scale, by + 15 * scale);
		Lift(LIFT_STATE::ON_DRAW);
		circleAntiClockwise(bx + 4 * scale, by + 15 * scale, 0.1 * scale, 1, -1, 1);
		delay(200);
		Lift(LIFT_STATE::BETWEEN_NUMS);
		GoTo(bx + 5 * scale, by + 5 * scale);
		Lift(LIFT_STATE::ON_DRAW);
		circleAntiClockwise(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
		delay(200);
		Lift(LIFT_STATE::BETWEEN_NUMS);
		break;

	}
}

void Control::Lift(LIFT_STATE s)
{
	int target;

	switch (s)
	{
	case LIFT_STATE::ON_DRAW:
		target = LIFT0;
		break;
	case LIFT_STATE::BETWEEN_NUMS:
		target = LIFT1;
		break;
	case LIFT_STATE::LIFTED:
		target = LIFT2;
		break;
	case LIFT_STATE::CLEAR:
		target = LIFT3;
	}

	if (servoLift > target)
	{
		while (servoLift > target)
		{
			servoLift--;
			servo_lift.writeMicroseconds(servoLift);
			delayMicroseconds(LIFTSPEED);
		}
    delay(50);
	}
	else
	{
		while (servoLift < target)
		{
			servoLift++;
			servo_lift.writeMicroseconds(servoLift);
			delayMicroseconds(LIFTSPEED);
		}
    delay(50);
	}
}

void Control::circleClockwise(float bx, float by, float radius, int start, int ende, float sqee)
{
	inkr = -0.05;
	count = 0;

	do
	{
		GoTo(sqee * radius * cos(start + count) + bx,
			radius * sin(start + count) + by);
		count += inkr;
	} while ((start + count) > ende);
  delay(50);
}

void Control::circleAntiClockwise(float bx, float by, float radius, int start, int ende, float sqee)
{
	inkr = 0.05;
	count = 0;

	do
	{
		GoTo(sqee * radius * cos(start + count) + bx,
			radius * sin(start + count) + by);
		count += inkr;
	} while ((start + count) <= ende);
  delay(50);
}

void Control::GoTo(float pX, float pY)
{
	float dx, dy, c;
	int i;

	// dx dy of new point
	dx = pX - lastX;
	dy = pY - lastY;
	//path lenght in mm, times 4 equals 4 steps per mm
	c = (float)floor(3 * sqrt(dx * dx + dy * dy));

	if (c < 1) c = 1;

	for (i = 0; i <= c; i++)
	{
		// draw line point by point
		set_XY(lastX + (i * dx / c), lastY + (i * dy / c));
	}

	lastX = pX;
	lastY = pY;
}


float Control::return_angle(float a, float b, float c)
{
	// cosine rule for angle between c and a
	return acos((a * a + c * c - b * b) / (2 * a * c));
}

void Control::set_XY(float Tx, float Ty)
{
	delay(10);
	float dx, dy, c, a1, a2, Hx, Hy;

	// calculate triangle between pen, servoLeft and arm joint
	// cartesian dx/dy
	dx = Tx - O1X;
	dy = Ty - O1Y;

	// polar lemgth (c) and angle (a1)
	c = sqrt(dx * dx + dy * dy); // 
	a1 = atan2(dy, dx); //
	a2 = return_angle(L1, L2, c);

	servo_left.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVOFAKTORLEFT) + SERVOLEFTNULL));

	// calculate joinr arm point for triangle of the right servo arm
	a2 = return_angle(L2, L1, c);
	Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); 
	Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI);

	// calculate triangle between pen joint, servoRight and arm joint
	dx = Hx - O2X;
	dy = Hy - O2Y;

	c = sqrt(dx * dx + dy * dy);
	a1 = atan2(dy, dx);
	a2 = return_angle(L1, (L2 - L3), c);

	servo_right.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTORRIGHT) + SERVORIGHTNULL));
}
