echo "Configuring and building ORB_modify ..."

mkdir build
cd build
cmake ..
make -j4
cd ..
# ./bin/orb_modify TUM2.yaml data/1.png

# ./bin/orb_modify TUM2.yaml data/oxford/bark/img1.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/bark/img2.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/bark/img3.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/bark/img4.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/bark/img5.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/bark/img6.ppm

# ./bin/orb_modify TUM2.yaml data/oxford/bikes/img1.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/bikes/img2.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/bikes/img3.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/bikes/img4.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/bikes/img5.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/bikes/img6.ppm

# ./bin/orb_modify TUM2.yaml data/oxford/graf/img1.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/graf/img2.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/graf/img3.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/graf/img4.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/graf/img5.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/graf/img6.ppm

# ./bin/orb_modify TUM2.yaml data/oxford/leuven/img1.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/leuven/img2.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/leuven/img3.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/leuven/img4.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/leuven/img5.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/leuven/img6.ppm
#
# ./bin/orb_modify TUM2.yaml data/oxford/trees/img1.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/trees/img2.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/trees/img3.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/trees/img4.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/trees/img5.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/trees/img6.ppm
#
# ./bin/orb_modify TUM2.yaml data/oxford/ubc/img1.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/ubc/img2.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/ubc/img3.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/ubc/img4.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/ubc/img5.ppm
# ./bin/orb_modify TUM2.yaml data/oxford/ubc/img6.ppm

# ./bin/orb_matcher TUM2.yaml data/1.png data/2.png
#./bin/orb_matcher TUM2.yaml data/1403636579763555584.png data/1403636579813555456.png
# ./bin/orb_matcher TUM2.yaml data/1403636579763555584.png data/1403636579913555456.png
# ./bin/orb_matcher TUM2.yaml data/1305031102.175304.png  data/1305031103.743326.png./bin/orb_matcher TUM2.yaml ./bin/orb_matcher TUM2.yaml ./bin/orb_matcher TUM2.yaml

./bin/orb_matcher TUM2.yaml data/oxford/bark/img1.ppm data/oxford/bark/img2.ppm bark12
./bin/orb_matcher TUM2.yaml data/oxford/bark/img1.ppm data/oxford/bark/img3.ppm bark13
./bin/orb_matcher TUM2.yaml data/oxford/bark/img1.ppm data/oxford/bark/img4.ppm bark14
./bin/orb_matcher TUM2.yaml data/oxford/bark/img1.ppm data/oxford/bark/img5.ppm bark15
./bin/orb_matcher TUM2.yaml data/oxford/bark/img1.ppm data/oxford/bark/img6.ppm bark16

./bin/orb_matcher TUM2.yaml data/oxford/bikes/img1.ppm data/oxford/bikes/img2.ppm bikes12
./bin/orb_matcher TUM2.yaml data/oxford/bikes/img1.ppm data/oxford/bikes/img3.ppm bikes13
./bin/orb_matcher TUM2.yaml data/oxford/bikes/img1.ppm data/oxford/bikes/img4.ppm bikes14
./bin/orb_matcher TUM2.yaml data/oxford/bikes/img1.ppm data/oxford/bikes/img5.ppm bikes15
./bin/orb_matcher TUM2.yaml data/oxford/bikes/img1.ppm data/oxford/bikes/img6.ppm bikes16

./bin/orb_matcher TUM2.yaml data/oxford/boat/img1.pgm data/oxford/boat/img2.pgm boat12
./bin/orb_matcher TUM2.yaml data/oxford/boat/img1.pgm data/oxford/boat/img3.pgm boat13
./bin/orb_matcher TUM2.yaml data/oxford/boat/img1.pgm data/oxford/boat/img4.pgm boat14
./bin/orb_matcher TUM2.yaml data/oxford/boat/img1.pgm data/oxford/boat/img5.pgm boat15
./bin/orb_matcher TUM2.yaml data/oxford/boat/img1.pgm data/oxford/boat/img6.pgm boat16

./bin/orb_matcher TUM2.yaml data/oxford/graf/img1.ppm data/oxford/graf/img2.ppm graf12
./bin/orb_matcher TUM2.yaml data/oxford/graf/img1.ppm data/oxford/graf/img3.ppm graf13
./bin/orb_matcher TUM2.yaml data/oxford/graf/img1.ppm data/oxford/graf/img4.ppm graf14
./bin/orb_matcher TUM2.yaml data/oxford/graf/img1.ppm data/oxford/graf/img5.ppm graf15
./bin/orb_matcher TUM2.yaml data/oxford/graf/img1.ppm data/oxford/graf/img6.ppm graf16

./bin/orb_matcher TUM2.yaml data/oxford/leuven/img1.ppm data/oxford/leuven/img2.ppm leuven12
./bin/orb_matcher TUM2.yaml data/oxford/leuven/img1.ppm data/oxford/leuven/img3.ppm leuven13
./bin/orb_matcher TUM2.yaml data/oxford/leuven/img1.ppm data/oxford/leuven/img4.ppm leuven14
./bin/orb_matcher TUM2.yaml data/oxford/leuven/img1.ppm data/oxford/leuven/img5.ppm leuven15
./bin/orb_matcher TUM2.yaml data/oxford/leuven/img1.ppm data/oxford/leuven/img6.ppm leuven16

./bin/orb_matcher TUM2.yaml data/oxford/trees/img1.ppm data/oxford/trees/img2.ppm trees12
./bin/orb_matcher TUM2.yaml data/oxford/trees/img1.ppm data/oxford/trees/img3.ppm trees13
./bin/orb_matcher TUM2.yaml data/oxford/trees/img1.ppm data/oxford/trees/img4.ppm trees14
./bin/orb_matcher TUM2.yaml data/oxford/trees/img1.ppm data/oxford/trees/img5.ppm trees15
./bin/orb_matcher TUM2.yaml data/oxford/trees/img1.ppm data/oxford/trees/img6.ppm trees16

./bin/orb_matcher TUM2.yaml data/oxford/ubc/img1.ppm data/oxford/ubc/img2.ppm oxford12
./bin/orb_matcher TUM2.yaml data/oxford/ubc/img1.ppm data/oxford/ubc/img3.ppm oxford13
./bin/orb_matcher TUM2.yaml data/oxford/ubc/img1.ppm data/oxford/ubc/img4.ppm oxford14
./bin/orb_matcher TUM2.yaml data/oxford/ubc/img1.ppm data/oxford/ubc/img5.ppm oxford15
./bin/orb_matcher TUM2.yaml data/oxford/ubc/img1.ppm data/oxford/ubc/img6.ppm oxford16

./bin/orb_matcher TUM2.yaml data/oxford/wall/img1.ppm data/oxford/wall/img2.ppm wall12
./bin/orb_matcher TUM2.yaml data/oxford/wall/img1.ppm data/oxford/wall/img3.ppm wall13
./bin/orb_matcher TUM2.yaml data/oxford/wall/img1.ppm data/oxford/wall/img4.ppm wall14
./bin/orb_matcher TUM2.yaml data/oxford/wall/img1.ppm data/oxford/wall/img5.ppm wall15
./bin/orb_matcher TUM2.yaml data/oxford/wall/img1.ppm data/oxford/wall/img6.ppm wall16