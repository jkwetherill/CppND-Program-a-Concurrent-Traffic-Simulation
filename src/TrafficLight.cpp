#include <iostream>
#include <future>
#include <random>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */


template <typename T>
T MessageQueue<T>::receive()
{
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 

    std::unique_lock<std::mutex> lock(_mutex);

    _cond.wait(lock);
 
    T msg = std::move(_queue.back());
    _queue.pop_back();
    return msg; // will not be copied due to return value optimization
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.
    std::lock_guard<std::mutex> lock(_mutex);

    //add a new message to the queue
    _queue.emplace_back(msg);

    //send a notification
    _cond.notify_one();
}


/* Implementation of class "TrafficLight" */


TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

void TrafficLight::waitForGreen()
{
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop
    // runs and repeatedly calls the receive function on the message queue.
    // Once it receives TrafficLightPhase::green, the method returns.

    while(_messageQueue.receive() != TrafficLightPhase::green){}

}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}

void TrafficLight::simulate()
{
    // FP.2b : Finally, the private method „cycleThroughPhases“ should be started in a thread when the public method „simulate“ is called. To do this, use the thread queue in the base class.

    // cycleThroughPhases();
    //  request entry to the current intersection (using async)
    std::future<void> F = std::async(std::launch::async, &TrafficLight::cycleThroughPhases, this);

    F.get();
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    // FP.2a : Implement the function with an infinite loop that measures the time between two loop cycles
    // and toggles the current phase of the traffic light between red and green and sends an update method
    // to the message queue using move semantics. The cycle duration should be a random value between 4 and 6 seconds.
    // Also, the while-loop should use std::this_thread::sleep_for to wait 1ms between two cycles.

    auto ti = std::chrono::system_clock::now();
    auto ti_light = ti; // beginning of phase cycle

    // std::chrono::time_point<std::chrono::system_clock> lastUpdate;

    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<> distr(4000, 6000);


    long cycle_dur_ms = distr(eng);
    // nextStreet = streetOptions.at(distr(eng));

    // init stop watch
    // lastUpdate = std::chrono::system_clock::now();
    // long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();

    while (true)
    { 
        //constant time step is 1 ms 
     const long time_step_ms = 1;   
     const auto time_step = std::chrono::milliseconds(time_step_ms);


      std::this_thread::sleep_for(time_step);

      // determine the elapsed time in ms
      auto tf = std::chrono::system_clock::now(); // current time
      auto dt = tf - ti; // elapsed time since beginning of interval
      auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(); //elapsed interval in ms

      if (dt_ms < time_step_ms)
      {
        continue;
      }
      
      //store current time as beginning of next interval
      ti = tf;

      auto dt_light = tf - ti_light;
      auto dt_light_ms = std::chrono::duration_cast<std::chrono::milliseconds>(dt_light).count();


      if (dt_light_ms < cycle_dur_ms)
            continue;
      
    
      //change the phase of the traffic light
      switch (_currentPhase){
        case TrafficLightPhase::red:
        {
            _currentPhase = TrafficLightPhase::green;
            break;
        }
        case TrafficLightPhase::green:
        {
            _currentPhase = TrafficLightPhase::red;
            break;
        }
        default:
        {

        }
      }  

      //send an update method to the message queue using move semantics
      _messageQueue.send(std::move(_currentPhase));

      cycle_dur_ms = distr(eng);

      //store current time as beginning of next interval
      ti = tf;
      ti_light = tf;
    }
}
