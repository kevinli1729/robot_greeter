#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

// using namespace cv;
// using namespace std;

static void read_csv(const std::string & filename, std::vector<cv::Mat> & images,
    std::vector<int> & labels, char separator = ';')
{
    std::ifstream file(filename.c_str(), std::ifstream::in);
    if (!file)
    {
        std::string error_message = "No valid input file";
        CV_Error(CV_StsBadArg, error_message);
    }
    std::string line, path, classlabel;
    while (getline(file, line))
    {
        std::stringstream liness(line);
        std::getline(liness, path, separator);
        std::getline(liness, classlabel);
        if (!path.empty() && !classlabel.empty())
        {
            images.push_back(cv::imread(path, 0));
            labels.push_back(std::atoi(classlabel.c_str()));
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "facial_recognizer");
    ros::start();

    std::string path = ros::package::getPath("robot_greeter");
    std::cout << "path to robot_greeter is: " << path << std::endl;

    std::string fn_haar = "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml";//string(argv[1]);
    // string fn_csv = "data/hello.txt";//string(argv[2]);
    int deviceId = 0;//atoi(argv[3]);

    /* testing for what I acutally inputed into the program
    cout << fn_haar << endl;
    cout << fn_csv << endl;
    cout << deviceId << endl;
    int wait;
    cin >> wait;
    return 0;
    */

    // These vectors hold the images and corresponding labels:
    std::vector<cv::Mat> images;
    std::vector<int> labels;

    // Check whether the training data file exists.
    // The training data file is called TrainingData.csv
    std::string trainingDataFile;

    // Read in the data (fails if no valid input filename is given):
    try
    {
        read_csv(trainingDataFile, images, labels);
    }
    catch (cv::Exception& e)
    {
        std::cerr << "Error opening file \"" << trainingDataFile << "\". Reason: " << e.msg << std::endl;
        // nothing more we can do
        exit(1);
    }
    // Get the height from the first image
    int im_width = images[0].cols;
    int im_height = images[0].rows;

    cv::CascadeClassifier haar_cascade;
    haar_cascade.load(fn_haar);
    // Get a handle to the Video device:
    cv::VideoCapture cap(deviceId);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 720);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    // Check if we can use this device at all:
    if (!cap.isOpened())
    {
        std::cerr << "Capture Device ID " << deviceId << "cannot be opened." << std::endl;
        return -1;
    }
    // Holds the current frame from the Video device:
    cv::Mat frame;
    std::string prev;
    while (true)
    {
        int humans, faces2;
        std::vector<std::string> humanNames;

        std::ifstream fin ("data/humans.txt");
        fin >> humans >> faces2;
        fin.close();

        std::ifstream fin2 ("data/names.txt");
        for (int i = 0; i < humans; i++)
        {
            std::string temp;
            std::getline(fin2, temp);
            humanNames.push_back(temp);
        }

        fin2.close();

        try
        {
            read_csv(trainingDataFile, images, labels);
        }
        catch (cv::Exception& e)
        {
            std::cerr << "Error opening file \"" << trainingDataFile << "\". Reason: " << e.msg << std::endl;
            // nothing more we can do
            exit(1);
        }

        // Create a FaceRecognizer and train it on the given  images:
        //Ptr<FaceRecognizer> model_1 = createLBPHFaceRecognizer(1, 8, 8, 8, 600);
        //Ptr<FaceRecognizer> model_2 = createFisherFaceRecognizer(0, 2000.0);
        cv::Ptr<cv::FaceRecognizer> model_3 = cv::createEigenFaceRecognizer(0, 5000.0);
        //model_1->train(images, labels);
        //model_2->train(images, labels);
        model_3->train(images, labels);

        cap >> frame;
        // Clone the current frame:
        cv::Mat original = frame.clone();
        // Convert the current frame to grayscale:
        cv::Mat gray;
        cvtColor(original, gray, CV_BGR2GRAY);
        // Find the faces in the frame:
        std::vector<cv::Rect_<int> > faces;
        haar_cascade.detectMultiScale(gray, faces, 1.1, 10);

        for(int i = 0; i < faces.size(); i++)
        {
            // Process face by face:
            cv::Rect face_i = faces[i];
            // Crop the face from the image
            cv::Mat face = gray(face_i);

            cv::Mat face_resized;
            cv::resize(face, face_resized, cv::Size(im_width, im_height), 1.0, 1.0, cv::INTER_CUBIC);
            // Now perform the prediction, see how easy that is:
            //int prediction_1 = model_1 -> predict(face_resized);
            //int prediction_2 = model_2 -> predict(face_resized);
            int prediction_3 = model_3 -> predict(face_resized);


            //cout << model -> predict(face_resized) << ' ';
            // And finally write all we've found out to the original image!
            // First of all draw a green rectangle around the detected face:
            rectangle(original, face_i, CV_RGB(0, 255,0), 1);
            // Create the text we will annotate the box with:
            //string box_text = format("%d, %d, %d", prediction_1, prediction_2, prediction_3);
            std::string box_text = cv::format("Predict: %d", prediction_3);
            // Calculate the position for annotated text (make sure we don't
            // put illegal values in there):
            int pos_x = std::max(face_i.tl().x - 10, 0);
            int pos_y = std::max(face_i.tl().y - 10, 0);
            // And now put it into the image:
            cv::putText(original, box_text, cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);

            // Show the result:
            cv::imshow("face_recognizer", original);

            char key = (char) cv::waitKey(5);
            if (prediction_3 == -1)
            {
                std::string human;
                std::cout << "Please enter your name: ";
                std::getline(std::cin, human);
                std::transform(human.begin(), human.end(), human.begin(), ::tolower);

                if (human.size() == 0)
                    human = prev;
                else
                    prev = human;
                int index = -1;
                for (int i = 0; i < humanNames.size(); i++)
                    if (humanNames[i] == human)
                    {
                        index = i;
                        break;
                    }

                faces2++;
                std::string str;
                std::ostringstream temp;
                temp << faces2;
                str = temp.str();
                std::string output = "data/images/" + human + '_' + str + ".jpg";
                imwrite(output, face_resized);

                if (index == -1)
                {
                    std::string str2;
                    humans++;
                    std::ostringstream temp2;
                    temp2 << humans - 1;
                    str2 = temp2.str();

                    std::ofstream fout ("data/names.txt", std::fstream::app);
                    fout << human << std::endl;
                    fout.close();

                    std::ofstream fout2 ("data/hello.txt", std::fstream::app);
                    fout2 << output + ';' + str2 << std::endl;
                    fout2.close();
                }
                else
                {
                    std::string str2;
                    std::ostringstream temp2;
                    temp2 << index;
                    str2 = temp2.str();
                    std::ofstream fout2 ("data/hello.txt", std::fstream::app);
                    fout2 << output + ';' + str2 << std::endl;
                    fout2.close();
                }

                std::ofstream fout3 ("data/humans.txt");
                fout3 << humans << ' ' << faces2 << std::endl;
            }


        }

        // And display it:
        char key = (char) cv::waitKey(5);
        // Exit this loop on escape:
        if (key == 27)
            break;
    }
    return 0;
}