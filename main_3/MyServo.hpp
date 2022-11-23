#include <Servo.h>
#include <Arduino.h>

class MyServo
{
private:
    Servo servo;
    float angle;
    float maximum;
    float minimum;
public:
    explicit MyServo(float starting_angle, float maximum, float minimum)
        : servo(),
          angle(starting_angle),
          minimum(minimum),
          maximum(maximum)
    {
    }

    void begin(size_t pin)
    {
        servo.attach(pin);
    }

    void change(float delta)
    {
        this->set(this->angle + delta);
    }

    void set(float angle)
    {
        if (angle > this->maximum)
        {
            this->angle = this->maximum;
        }
        else if (angle < this->minimum)
        {
            this->angle = this->minimum;
        }
        else
        {
            this->angle = angle;
        }
    }
    float get()
    {
        return this->angle;
    }

    void update()
    {
        this->servo.write(static_cast<int>(this->angle));
    }
};