
from aupyom import Sound
import sounddevice as sd
import pkg_resources
import sounddevice as sd


from aupyom import Sampler

def file1_audio_file():
    return pkg_resources.resource_filename(__name__, "Yamaha.wav")


audio_file = file1_audio_file()
print(audio_file)

s1 = Sound.from_file(audio_file)


sampler = Sampler()

sampler.play(s1)


print(s1.playing)
