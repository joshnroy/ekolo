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

    cout << "LOCKING MOOTEX" << endl;
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

    alGenSources((ALsizei)1, &source);
    this->sources = new ALuint[MAX_SOURCES];
    this->buffers = new ALuint[MAX_SOURCES];
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
    this->m.unlock();
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
    alGetError();

    cout << "thing" << endl;
    alDeleteSources(this->numPoints, this->sources);
    error = alGetError();
    cout << error << endl;
    cout << "1" << endl;
    
    alDeleteBuffers(this->numPoints, this->buffers);

    this->numPoints = pc.points.size();
    if (this->numPoints > MAX_SOURCES)
        this->numPoints = MAX_SOURCES;
    cout << "2" << endl;
    //cout << sources << ", " << numPoints << endl;


    alGenSources((ALsizei)MAX_SOURCES, this->sources);
    cout << "2.5" << endl;

    error = alGetError();
    cout << error << endl;
    cout << "2.6" << endl;
    if (error != AL_NO_ERROR) {
	assert(1 == 0);
        cout << "could not generate the sources";
        return;
    }
    cout << "2.75" << endl;

    alGenBuffers((ALuint)this->numPoints, this->buffers);
    // check for errors

    error = alGetError();
    if (error != AL_NO_ERROR) {
        cout << "could not generate the buffers";
        return;
    }

    cout << "2.95" << endl;

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
	cout << "hi " << i << endl;
        geometry_msgs::Point32 curPt = pc.points[i];

        // first we load data into the buffer
        alBufferData(this->buffers[i], format, data, size, freq);
        // check for errors
        error = alGetError();
        if (error != AL_NO_ERROR) {
            cout << "could not load c_buffer" << error << endl;
            return;
        }

        // then we assign the point's source a tone
        alSourcei(sources[i], AL_BUFFER, this->buffers[i]);
        // check for errors
        error = alGetError();
        if (error != AL_NO_ERROR) {
            cout << "could not assign buffer to source" << endl;
            return;
        }

        // then we set the source to the point's position
        alSourcef(sources[i], AL_PITCH, 1);
        alSourcef(sources[i], AL_GAIN, 1);
        alSource3f(sources[i], AL_POSITION, curPt.x * 100, curPt.y * 100, curPt.z * 100);
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
    cout << "unlocking" << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sound_library");

    SoundController sc;

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/slam/pointcloud", 1, &SoundController::displayPoints, &sc);
    ros::Subscriber camera_sub = n.subscribe("/slam/pos", 1, &SoundController::displayPoints, &sc);

    ros::spin();

    return 0;
}
