/* 
Copyright 2013-2016 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.  
*/


/*
 * rostime.h
 *
 * This header contains the definition of a ROST timestamp, plus some inline convenience functions 
 *
 * It also provides methods to sleep and get the current time using ROStime_t
 *
 */
#ifndef ROSTIME_HPP_
#define ROSTIME_HPP_

#ifndef _WIN32
 #include <time.h>
#endif
  /*
	ROS-compatible time definition used on this interface: 
	this is just boilerplate to avoid dependency on ROS
  */
  typedef struct {
    int32_t secs;  //time in seconds since unix epoch
    int32_t nsecs; //time in nanoseconds since last second
  } ROStime_t;


 inline const ROStime_t rectify(const ROStime_t& t) 
 {
 ROStime_t result = t;
 while (result.nsecs >= 1000000000) {
    result.secs++;
    result.nsecs -= 1000000000;
 }   
 while (result.nsecs < 0) {
    result.secs--;
    result.nsecs += 1000000000;
 }   
 return result;    
 }


 inline bool operator< (const ROStime_t l,const ROStime_t r) 
 {
     if (l.secs < r.secs)   {return 1;}
     if (l.secs > r.secs)   {return 0;}
     if (l.nsecs < r.nsecs) {return 1;}
     else                   {return 0;}     
 }     
 inline bool operator> (const ROStime_t lhs, const ROStime_t rhs){return rhs < lhs;}
 inline bool operator<=(const ROStime_t lhs, const ROStime_t rhs){return !(lhs > rhs);}
 inline bool operator>=(const ROStime_t lhs, const ROStime_t rhs){return !(lhs < rhs);}


 inline const ROStime_t operator+ (const ROStime_t lhs,const ROStime_t rhs) 
 {
     ROStime_t result;
     result.secs = lhs.secs + rhs.secs;
     result.nsecs = lhs.nsecs + rhs.nsecs;
     return rectify(result);
 }

 inline const ROStime_t operator- (const ROStime_t& lhs,const ROStime_t& rhs) 
 {
     ROStime_t result;
     result.secs = lhs.secs - rhs.secs;
     result.nsecs = lhs.nsecs - rhs.nsecs;
     return rectify(result);
 }

 inline const std::ostream& operator<< (std::ostream &strm, const ROStime_t& t)
 {
	 return strm << t.secs << "s " << t.nsecs << "ns";
 }

 inline const ROStime_t ConvertFromNS(long nsecs)
 {     
     ROStime_t result;
     result.nsecs = nsecs % 1000000000;
     result.secs = nsecs / 1000000000;
     return result;
 }

#ifndef _WIN32
 inline const ROStime_t convert_timespec(timespec t)
 {     
     ROStime_t result;
     result.nsecs = t.tv_nsec;
     result.secs = t.tv_sec;
     return result;
 }

 inline const timespec convert_timespec(ROStime_t t)
 {     
     timespec result; 
     result.tv_nsec = t.nsecs;
     result.tv_sec = t.secs;
     return result;
 }


/*
Get the current time
*/
inline ROStime_t GetNow()
{
 struct timespec ts1;
 clock_gettime(CLOCK_REALTIME, &ts1);
 return convert_timespec(ts1);
}

/*
 Check whether we can access the realtime clock
*/
inline void Sleep(ROStime_t sleep)
{
 if (sleep.secs < 0) return;
 struct timespec ts1, ts2;
 ts1 = convert_timespec(sleep);
 nanosleep(&ts1, &ts2);
 return;
}
#endif //#ifndef _WIN32

/*
 Check whether we can access the realtime clock
*/
inline bool IsClockAccessible()
{
#ifndef _WIN32
 struct timespec ts1;
 int err = clock_gettime(CLOCK_REALTIME, &ts1);
 return (err != -1);
#else
	return false;
#endif
}




#endif /* ROSTIME_HPP_ */
