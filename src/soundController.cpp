#include "ros/ros.h"
#include <iostream>
#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alut.h>
#include <unistd.h>
#include <mutex>

#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"

using namespace std;

#define MAX_SOURCES 100

class SoundController {

    public:
        void displayPoints(sensor_msgs::PointCloud pc);
        void updateCameraLocation(double ax, double ay, double az, double ux, double uy, double uz, double px, double py, double pz);
        SoundController();
        ~SoundController();
        // start variables
        ALuint *sources;
        ALuint numPoints;
        ALuint *buffers;
        ALCcontext *context;
        mutex m;
};

SoundController::SoundController() {

    this->m.lock();
    
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

    alGenSources((ALuint)1, &source);
    // check for errors


    error = alGetError();
    if (error != AL_NO_ERROR) {
        cout << "bad things happened while setting up the source";
        return;
    }


    this->m.unlock();

}

SoundController::~SoundController() {
    this->m.lock();
	alDeleteSources(this->numPoints, this->sources);
    alDeleteBuffers(this->numPoints, this->buffers);
    ALCdevice *device = alcGetContextsDevice(context);
    alcMakeContextCurrent(NULL);
    alcDestroyContext(context);
    alcCloseDevice(device);
}

void updateCameraLocation(double ax, double ay, double az, double ux, double uy, double uz, double px, double py, double pz) {
    ALfloat listenerOri[] = { ax, ay, az, ux, uy, uz };

    alListener3f(AL_POSITION, px, py, pz);
    alListenerfv(AL_ORIENTATION, listenerOri);

    error = alGetError();
    if (error != AL_NO_ERROR) {
        cout << "There was an error while moving the listener!" << endl;
        return;
    }
}

void SoundController::displayPoints(sensor_msgs::PointCloud pc) {

    this->m.lock();

    // this will store the error results
    ALCenum error;

    cout << "thing" << endl;
    alDeleteSources(this->numPoints, this->sources);
    cout << "1" << endl;

    this->numPoints = pc.points.size();
    if (this->numPoints > MAX_SOURCES)
        this->numPoints = MAX_SOURCES;
    cout << "2" << endl;
    //cout << sources << ", " << numPoints << endl;


    ALuint sourcesArray[this->numPoints];
    sources = sourcesArray;
    alGenSources((ALsizei)this->numPoints, sources);

    error = alGetError();
    if (error != AL_NO_ERROR) {
        cout << "could not generate the sources";
        return;
    }

    ALuint buffersArray[this->numPoints];
    buffers = buffersArray;
    alGenBuffers((ALuint)this->numPoints, buffers);
    // check for errors

    error = alGetError();
    if (error != AL_NO_ERROR) {
        cout << "could not generate the buffers";
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


    // transform the Point32s into audio sources
    for (int i = 0; i < numPoints; i++) {
        geometry_msgs::Point32 curPt = pc.points[i];

        // first we load data into the buffer
        alBufferData(buffers[i], format, data, size, freq);
        // check for errors
        error = alGetError();
        if (error != AL_NO_ERROR) {
            cout << "could not load c_buffer" << error << endl;
            return;
        }

        // then we assign the point's source a tone
        alSourcei(sources[i], AL_BUFFER, buffers[i]);
        // check for errors
        error = alGetError();
        if (error != AL_NO_ERROR) {
            cout << "could not assign buffer to source" << endl;
            return;
        }

        // then we set the source to the point's position
        alSourcef(sources[i], AL_PITCH, 1);
        alSourcef(sources[i], AL_GAIN, 1);
        alSource3f(sources[i], AL_POSITION, curPt.x, curPt.y, curPt.z);
        alSource3f(sources[i], AL_VELOCITY, 0, 0, 0);
        alSourcei(sources[i], AL_LOOPING, AL_TRUE);
        error = alGetError();
        if (error != AL_NO_ERROR) {
            cout << "could not set source properties" << endl;
            return;
        }


        // start sound from the source
        alSourcePlay(sources[i]);
        error = alGetError();
        if (error != AL_NO_ERROR) {
            cout << "could not play source" << endl;
            return;
        }
    }
    cout << "4" << endl;

    cout << "finished" << endl;
    this->m.unlock();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sound_library");

    SoundController sc;

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/slam/pointcloud", 1, &SoundController::displayPoints, &sc);

    ros::spin();

    return 0;
}
