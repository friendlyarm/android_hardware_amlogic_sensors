void ComputeAndOrientation(float heading, float euler[3], float* result) {
    float pitch, roll;

    result[0] = heading;

    pitch = euler[1];
    roll  = euler[0];

    //normalize pitch and roll
    if(pitch < 0.0f) {
        pitch += 360.0f;
    }

    if(roll < 0.0f) {
        roll += 360.0f;
    }

    //android weirdness
    if(roll > 90.0f && roll < 270.0f) {
        pitch -=180.0f;
        if(pitch < 0.0f) {
            pitch += 360.0f;
        }
    }
    //set pitch
    result[1] = pitch;

    if(roll >= 0.0f && roll <= 90.0f) {
        result[2] = -roll;
    } else if(roll > 90.0f && roll < 270.0f) {
        result[2] = roll - 180.0f;
    } else { // roll >=  270.0f
        result[2] = 360.0f - roll;
    }

}
