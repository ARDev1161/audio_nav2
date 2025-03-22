# my_msgs/msg/SoundSourceClassification.msg

std_msgs/Header header

int64 id         # ID источника (если взяли из /sst)
float64 x
float64 y
float64 z

string label     # класс звука (Speech, Music, etc)
float64 confidence

float64 energy   # e из /ssl или activity из /sst
