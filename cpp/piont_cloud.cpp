#include <iostream>
#include <cmath> 

using namespace std; 

int main() {
    double angle =24.8;
    long long speed_of_light = 299792458;
    double roundtrip_time = 66.7 * pow(10, -9);
    float single_dist = roundtrip_time/2;
    double d = single_dist*speed_of_light;
    double angle_adjusted = (90-angle)*M_PI/180;

    cout<<"Distance (hypotenuse) = "<<d<<"; X=d*sin(t)= "<<d*sin(angle_adjusted)<<"; Y= 0; Z=d*sin(t)= "<<d*cos(angle_adjusted)<<endl;
}