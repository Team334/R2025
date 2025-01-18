# all inputs
echo -n "Enter camera name (ex: arducam-1): "
read -r name

echo -n "Enter calibration resolution (ex: 1280x720): "
read -r resolution

echo -n "Enter input video file path (ex: /path/to/input.mkv): "
read -r input

echo -n "Enter desired ffmpeg fps (ex: 2): "
read -r fps

echo -n "Enter grid size (ex: 14): "
read -r n

echo -n "Enter grid spacing in meters (ex: 0.012): "
read -r spacing

echo -n "Enter focal length in pixels (ex: 1015): "
read -r focal

mrcal_to_photon="$PWD/mrcal_to_photon.py"

# cd to where the input file is
cd "$(dirname "$input")"

# rename input to just the file name
input=$(basename "$input")

# calib directory name
calibdir=$name/$resolution

# make calib directory
mkdir -p $calibdir
cd $calibdir

# move input into calib directory and create frame directory
mv "../../$input" .
mkdir -p frames

# fill up frame directory with png frames
ffmpeg -i $input -vf fps=$fps frames/%04d.png

# generate corners.vnl
echo Generating corners.vnl file...
mrgingham --jobs 8 --gridn $n 'frames/*.png' > corners.vnl 

# set up calibration globs
evens='frames/*[02468].png'
odds='frames/*[13579].png'
all='frames/*.png'

mrcal_calibrate=(
    mrcal-calibrate-cameras \
        --corners-cache corners.vnl \
        --lensmodel LENSMODEL_OPENCV8 \
        --focal $focal \
        --object-spacing $spacing \
        --object-width-n $n \
)

echo Running all calibrations...

# cross diff calibration
$mrcal_calibrate $evens
mv camera-0.cameramodel $name-evens.cameramodel

$mrcal_calibrate $odds
mv camera-0.cameramodel $name-odds.cameramodel

# main calibration
$mrcal_calibrate $all
mv camera-0.cameramodel $name.cameramodel

# turn mrcal cameramodel to photon json
echo Turning cameramodel into json...
python3 $mrcal_to_photon $name.cameramodel "$name ($resolution).json"

echo Finished! You should now analyze the calibration as described in the mrcal docs.

# done!
