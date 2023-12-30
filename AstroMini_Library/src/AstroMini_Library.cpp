/*
 *  This program calculates solar positions as a function of location, date, and time.
 *  The equations are from Jean Meeus, Astronomical Algorithms, Willmann-Bell, Inc., Richmond, VA
 *  (C) 2015, David Brooks, Institute for Earth Science Research and Education.
 */

#include <Time.h>
#include <time.h>
#include <math.h>


#include "AstroMini_Library.h"


AstroMini::AstroMini (int hr, int min, int sec, int z,
		      int m, int d, int y,
		      float lon, float lat){
  // init time, date, location
  ahour=hr; aminute=min; asecond=sec; zone=z;
  amonth=m; aday=d; ayear=y;
  Lon=lon; Lat=lat; ms0=0;
}

void  AstroMini::Set_Time (int hr, int min, int sec, unsigned long millis0, int z){
  ahour=hr; aminute=min; asecond=sec; zone=z; ms0=millis0;
}

void  AstroMini::Set_Date (int y, int m, int d){
  ayear=y; amonth=m; aday=d; 
}
  
void AstroMini::Set_Location (float lon, float lat){
  Lon=lon; Lat=lat;
}

#ifdef ASTROMINI_ALL
void AstroMini::Test (HardwareSerial *s, unsigned long ms){
    calc_JD_Time_GrHrAngle (ms);
    sun_pos (); // test with sun pos
    calc_HrAngle ();
    s->println("-JD-");
    // TEST
    s->print(ayear); s->print(","); s->print(amonth);
    s->print(","); s->print(aday); s->print(", ");
    s->print(ahour); s->print(",");
    s->print(aminute); s->print(","); s->print(asecond+ms/1000.);
    // (Optional) display results of intermediate calculations->
    s->print(", JD="); s->print(JD_whole);
    s->print(","); s->print(JD_frac,7);
    //s->print(","); s->print(T,7);
    //s->print(","); s->print(L0,7);
    //s->print(","); s->print(M,7);
    //s->print(","); s->print(e,7);
    //s->print(","); s->print(C,7);
    //s->print(","); s->print(L_true,7);
    //s->print(","); s->print(f,7);
    //s->print(","); s->print(R,7);
    s->print(", GrH="); s->print(GrHrAngle,7);
    s->println();
    s->println("-SUN-");
    //s->print(","); Ser+ial.print(Obl,7);
    s->print("RA="); s->print(RA,7);
    s->print(", DE="); s->print(Decl,7);
    s->print(", HrA="); s->print(HrAngle,7);
    s->print(", Elev="); s->print(calc_Elevation ()/DEG_TO_RAD,3);
    s->print(", Az="); s->print(calc_Azimuth ()/DEG_TO_RAD,3);
    s->println();

    s->println("-ArduinoTime-");
    s->print(ms);
    s->println("ms");

    /*
    s->println("-ArduinoTime-");
    s->print(hour()); s->print(":"); s->print(minute()); s->print(":"); s->print(second());
    s->print(" ");
    s->print(day());
    s->print(". ");
    s->print(month());
    s->print(". ");
    s->print(year());
    s->println();
    */
}


void AstroMini::sun_pos (){
    L0=DEG_TO_RAD*fmod(280.46645+36000.76983*T,360);
    M=DEG_TO_RAD*fmod(357.5291+35999.0503*T,360);
    e=0.016708617-0.000042037*T;
    C=DEG_TO_RAD*((1.9146-0.004847*T)*sin(M)+(0.019993-0.000101*T)*sin(2*M)+0.00029*sin(3*M));
    f=M+C;
    Obl=DEG_TO_RAD*(23+26/60.+21.448/3600.-46.815/3600*T);

    L_true=fmod(C+L0,TWOPI);
    R=1.000001018*(1-e*e)/(1+e*cos(f));

    RA   = atan2(sin(L_true)*cos(Obl),cos(L_true));
    Decl = asin(sin(Obl)*sin(L_true));
};
#endif

float AstroMini::calc_RA_from_hrangle (float hrangle){ // in RAD!!
    RA = GrHrAngle-Lon-hrangle;
    if (RA < 0)
      RA += TWOPI;
    return RA; // in RAD
}
  
void AstroMini::calc_JD_Time_GrHrAngle (unsigned long ms){
    ms -= ms0;
    JD_whole=JulianDate(ayear,amonth,aday);
    JD_frac=(ahour+zone+aminute/60.+asecond/3600.+ms/3600000.)/24.-.5;
    T=JD_whole-2451545; T=(T+JD_frac)/36525.;

    JDx=JD_whole-2451545;
    GrHrAngle=280.46061837+(360*JDx)%360+.98564736629*JDx+360.98564736629*JD_frac;
    GrHrAngle=DEG_TO_RAD*fmod(GrHrAngle,360.); // RAD now!
}

float AstroMini::calc_HrAngle (){
    // now update with RA, Decl
    HrAngle=GrHrAngle-Lon-RA; // East Longitudes are expressed as negative
    if (HrAngle < 0)
      HrAngle += TWOPI;
    return HrAngle;
}

float AstroMini::calc_Elevation (){
    return asin(sin(Lat)*sin(Decl)+cos(Lat)*(cos(Decl)*cos(HrAngle)));
}
float AstroMini::calc_Azimuth (){
    return PI+atan2(sin(HrAngle),cos(HrAngle)*sin(Lat)-tan(Decl)*cos(Lat));
}
  
long AstroMini::JulianDate(int y, int m, int d) {
    long JD_whole;
    int A,B;
    if (m<=2) {
      y--; m+=12;
    }
    A=y/100; B=2-A+A/4;
    JD_whole=(long)(365.25*(y+4716))+(int)(30.6001*(m+1))+d+B-1524;
    return JD_whole;
}
