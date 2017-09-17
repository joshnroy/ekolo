#include "ros/ros.h"
#include <iostream>
#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alut.h>
#include <unistd.h>

#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"

using namespace std;

#define MAX_SOURCES 100

class SoundController {

    public:
        void displayPoints(sensor_msgs::PointCloud pc);
        SoundController();
        ~SoundController();
        // start variables
        ALuint *sources;
        ALuint numSources;
        ALuint c_buffer;
        ALCcontext *context;
	int maxSources;
};

SoundController::SoundController() {

    maxSources = 10000;
    
    ALCdevice *device;

    device = alcOpenDevice(NULL);
    if (!device) {
        cout << "There's a problem setting up the device" << endl;
    }

    ALboolean enumeration;

    enumeration = alcIsExtensionPresent(NULL, "ALC_ENUMERATION_EXT");
    if (enumeration == AL_FALSE) {
        // enumeration not supported
        cout << "Enumeration not supported";
        return;
    }
    //else enumeration supported
    ALCenum error;

    context = alcCreateContext(device, NULL);
    if (!alcMakeContextCurrent(context)) {
        cout << "Could not make audio context!";
        return;
    }
    // test for errors here using alGetError();

    ALfloat listenerOri[] = { 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f };

    alListener3f(AL_POSITION, 0, 0, 1.0f);
    // check for errors
    alListener3f(AL_VELOCITY, 0, 0, 0);
    // check for errors
    alListenerfv(AL_ORIENTATION, listenerOri);
    // check for errors

    error = alGetError();
    if (error != AL_NO_ERROR) {
        cout << "bad things happened while setting up the listener";
        return;
    }

    ALuint source;

    cout << "test" << endl;
//  alGenSources((ALuint)1, &source);
    ALuint sourcesArray[maxSources];
    sources = sourcesArray;
    alGenSources((ALsizei)maxSources, sources);
    cout << "close test" << endl;
    // check for errors


    error = alGetError();
    cout << "a" << endl;
    cout << (error != AL_NO_ERROR) << endl;
    cout << "b" << endl;
    if (error != AL_NO_ERROR) {
        cout << "bad things happened while setting up the source";
        return;
    }
    cout << "dsiojf" << endl;
    

    alGenBuffers((ALuint)1, &c_buffer);
    // check for errors

    error = alGetError();
    if (error != AL_NO_ERROR) {
        cout << "could not generate the buffer";
        return;
    }

    ALsizei size, freq;
    ALenum format;
    ALvoid *data;
    ALboolean loop = AL_FALSE;

    alutLoadWAVFile((ALbyte*)"c.wav", &format, &data, &size, &freq, &loop);
    error = alGetError();
    if (error != AL_NO_ERROR) {
        cout << "could not load .wav file" << endl;
        return;
    }
    cout << "loaded .wav file successfully" << endl;
    alBufferData(c_buffer, format, data, size, freq);
    // check for errors
    error = alGetError();
    if (error != AL_NO_ERROR) {
        cout << "could not load c_buffer" << error << endl;
        return;
    }

}

SoundController::~SoundController() {
	alDeleteSources(this->numSources, this->sources);
    alDeleteBuffers(1, &c_buffer);
    ALCdevice *device = alcGetContextsDevice(context);
    alcMakeContextCurrent(NULL);
    alcDestroyContext(context);
    alcCloseDevice(device);
}

void SoundController::displayPoints(sensor_msgs::PointCloud pc) {

//  alDeleteSources(this->numSources, this->sources);
    cout << "1" << endl;

//  int numPoints = pc.points.size();
//  if (numPoints > MAX_SOURCES) {
//      numPoints = MAX_SOURCES;
//  }
    cout << "2" << endl;

    cout << "3" << endl;

//  // transform the Point32s into audio sources
//  for (int i = 0; i < numPoints; i++) {
//      geometry_msgs::Point32 curPt = pc.points[i];

//      // first we assign the point's source a tone
//      alSourcei(sources[i], AL_BUFFER, c_buffer);
//      // check for errors
//      ALCenum error;
//      error = alGetError();
//      if (error != AL_NO_ERROR) {
//          cout << "could not assign buffer to source" << endl;
//          return;
//      }

//      // then we set the source to the point's position
//      alSourcef(sources[i], AL_PITCH, 1);
//      alSourcef(sources[i], AL_GAIN, 1);
//      alSource3f(sources[i], AL_POSITION, curPt.x, curPt.y, curPt.z);
//      alSource3f(sources[i], AL_VELOCITY, 0, 0, 0);
//      alSourcei(sources[i], AL_LOOPING, AL_TRUE);
//      error = alGetError();
//      if (error != AL_NO_ERROR) {
//          cout << "could not set source properties" << endl;
//          return;
//      }

//      // start sound from the source
//      alSourcePlay(sources[i]);
//      error = alGetError();
//      if (error != AL_NO_ERROR) {
//          cout << "could not play source" << endl;
//          return;
//      }
//  }
    cout << "4" << endl;

    cout << "finished" << endl;

    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sound_controller_moving");

    SoundController sc;

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/slam/pointcloud", 1, &SoundController::displayPoints, &sc);

    ros::spin();

    return 0;
}
