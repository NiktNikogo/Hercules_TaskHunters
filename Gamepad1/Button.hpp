#include <stdint.h>
#include <Arduino.h>
class Button {
private:
    bool state;
    uint64_t last_clicked;
    bool clicked;
protected: 
    size_t pin;
    virtual bool read() = 0 ;
public:
    static constexpr uint64_t DEBOUNCE_TIME = 200;
public:
    explicit Button(size_t pin)
    : 
    state( false ),
    last_clicked( 0 ),
    pin(pin)
    {
        pinMode(this->pin, INPUT_PULLUP);
    }
    bool get_state() {
        return this->state;
    }
    void update(uint64_t current_time) {
        if(current_time - this->last_clicked >= Button::DEBOUNCE_TIME) {
            auto button_state = this->read();
            if( !this->clicked && button_state ) {
                this->state = !this->state;    
                this->last_clicked = current_time;   
            }
            this->clicked = button_state;
        }
    }
};

class Digital_button : public Button {
public:
    Digital_button(size_t pin) :
        Button(pin)
    {}
protected:
    virtual bool read() override {
        return !digitalRead(this->pin);
    }
};

class Analog_button : public Button{
public:
    Analog_button(size_t pin) :
        Button(pin)
    {}
protected:
    virtual bool read() override {
        return analogRead(this->pin) == 0;
    }
};