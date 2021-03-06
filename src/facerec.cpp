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

#include <sys/types.h>
#include <sys/stat.h>

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Char.h"

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

/*!
 * Check whether the data directory exists and whether all necessary files
 * within it exist. Creates the directory and the necessary files if they
 * do not exist.
 *
 * @return whether there are two images in the data/images/ directory.
 */
inline bool initData()
{
    // Get path to the robot_greeter package
    std::string robotGreeterPath = ros::package::getPath("robot_greeter");

    // Check if a data directory exists
    {
        struct stat info;
        std::string folderPath = robotGreeterPath + "/data";

        bool folderExists = true;
        if (stat(folderPath.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR))
            folderExists = false;

        if (!folderExists)
            mkdir(folderPath.c_str(), S_IRWXU);
    }

    // Check if a data/images directory exists
    {
        struct stat info;
        std::string folderPath = robotGreeterPath + "/data/images";

        bool folderExists = true;
        if (stat(folderPath.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR))
            folderExists = false;

        if (!folderExists)
            mkdir(folderPath.c_str(), S_IRWXU);
    }

    // Check if training data file exists. Create it if it does not, it creates it.
    {
        std::string trainingDataFile = robotGreeterPath + "/data/TrainingData.csv";
        std::ifstream file(trainingDataFile.c_str(), std::ifstream::in);
        if (!file)
        {
            // Create the training data file
            std::ofstream fout2 (trainingDataFile.c_str());
            fout2.close();
        }
    }

    // check if a data/humans.txt file exists. Create it if it does not. (contains two integers separated by a space)
    {
        std::string humanDataFile = robotGreeterPath + "/data/humans.txt";
        std::ifstream file(humanDataFile.c_str(), std::ifstream::in);
        if (!file)
        {
            // Create the training data file
            std::ofstream fout2 (humanDataFile.c_str());
            fout2 << "0 0" << std::endl;
            fout2.close();
        }
    }


    // check if a data/names.txt (list of human names, one line per name)
    {
        std::string namesDataFile = robotGreeterPath + "/data/names.txt";
        std::ifstream file(namesDataFile.c_str(), std::ifstream::in);
        if (!file)
        {
            // Create the training data file
            std::ofstream fout2 (namesDataFile.c_str());
            fout2.close();
        }
    }

    return true;
}

char ScanMsg;

void returnMessage(const std_msgs::Char::ConstPtr & message)
{
    //ROS_INFO("Facerec heard: [%s] from scanning application", message -> data);
    ScanMsg = message -> data;
}


int main(int argc, char** argv)
{
    //ros::start();
    ros::init(argc, argv, "facial_recognizer");
    //ros::init(argc, argv, "facial_recognizer_listener");

    if (!initData())
        exit(1);

    std::cout << "checkpoint1" << std::endl;
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

    std::string pathToRobotGreeter = ros::package::getPath("robot_greeter");
    std::string trainingDataFile = pathToRobotGreeter + "/data/TrainingData.csv";

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
    int im_width = 200;//images[0].cols;
    int im_height = 200;//images[0].rows;

    cv::CascadeClassifier haar_cascade;
    haar_cascade.load(fn_haar);
    // Get a handle to the Video device:
    cv::VideoCapture cap(deviceId);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 720); // the screen is currently 720x720
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

    int humans, faces2;
    std::ifstream fin3 ((pathToRobotGreeter + "/data/humans.txt").c_str());
    fin3 >> humans >> faces2;
    fin3.close();

    //cout << human << ' ' << faces2 << endl;

    ros::init(argc, argv, "talker");
    ros::NodeHandle n, n2, n3;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Publisher chatter2_pub = n2.advertise<std_msgs::Bool>("chatter2", 1000);

    ScanMsg = 'g';
    // the case where there are less than two different humans in the picture
    // it takes pictures of humans until it can register two different humans
    


    bool reco = false;
    while (humans < 2 && ros::ok)
    {
        {
            std_msgs::String msg;
            msg.data = "scan";
            chatter_pub.publish(msg);
            ros::spinOnce();
            ROS_INFO("%s", msg.data.c_str());
        }

        std::vector<std::string> humanNames;

        std::ifstream fin2 ((pathToRobotGreeter + "/data/names.txt").c_str());
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
        //cv::Ptr<cv::FaceRecognizer> model_3 = cv::createEigenFaceRecognizer(0, 5000.0);
        //model_1->train(images, labels);
        //model_2->train(images, labels);
        //model_3->train(images, labels);

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
            cv::Rect face_i = faces[i];

            cv::Point center = cv::Point((face_i.tl().x + face_i.br().x) * 0.5, (face_i.tl().y + face_i.br().y) * 0.5);

            std_msgs::String msg2;
            msg2.data = "focus_h";
            chatter_pub.publish(msg2);
            ros::spinOnce();

            std_msgs::Bool msg_b;
            int dif = center.x - 360;
            while (std::abs(dif) > 5 && ros::ok()) // false is move left
            {
                msg_b.data = dif > 0?false:true;
                chatter2_pub.publish(msg_b);
                ros::spinOnce();
                ros::Subscriber sub = n3.subscribe("fail", 1000, returnMessage);
                if (ScanMsg != 'g')
                    break;

                cap >> frame;
                cv::Mat original = frame.clone();
                // Convert the current frame to grayscale:
                cv::Mat gray;
                cvtColor(original, gray, CV_BGR2GRAY);
                // Find the faces in the frame:
                std::vector<cv::Rect_<int> > faces;
                haar_cascade.detectMultiScale(gray, faces, 1.1, 10);
                cv::Rect face_i = faces[i];
                cv::Point center = cv::Point((face_i.tl().x + face_i.br().x) * 0.5, (face_i.tl().y + face_i.br().y) * 0.5);
                dif = center.x - 360;
            }

            if (ScanMsg != 'g')
                break;

            msg2.data = "focus_v";
            chatter2_pub.publish(msg_b);
            ros::spinOnce();

            dif = center.y - 360;
            while (std::abs(dif) > 5 && ros::ok()) // false is move up
            {
                msg_b.data = dif > 0?false:true;
                chatter2_pub.publish(msg_b);
                ros::spinOnce();
                ros::Subscriber sub = n3.subscribe("fail", 1000, returnMessage);
                if (ScanMsg != 'g')
                    break;


                cap >> frame;
                cv::Mat original = frame.clone();
                // Convert the current frame to grayscale:
                cv::Mat gray;
                cvtColor(original, gray, CV_BGR2GRAY);
                // Find the faces in the frame:
                std::vector<cv::Rect_<int> > faces;
                haar_cascade.detectMultiScale(gray, faces, 1.1, 10);
                cv::Rect face_i = faces[i];
                cv::Point center = cv::Point((face_i.tl().x + face_i.br().x) * 0.5, (face_i.tl().y + face_i.br().y) * 0.5);
                dif = center.y - 360;
            }

            if (ScanMsg != 'g')
                break;

            // Process face by face:
            // Crop the face from the image
            cv::Mat face = gray(face_i);

            cv::Mat face_resized;
            cv::resize(face, face_resized, cv::Size(im_width, im_height), 1.0, 1.0, cv::INTER_CUBIC);

            //cout << model -> predict(face_resized) << ' ';
            // And finally write all we've found out to the original image!
            // First of all draw a green rectangle around the detected face:
            rectangle(original, face_i, CV_RGB(0, 255,0), 1);
            // Create the text we will annotate the box with:
            //string box_text = format("%d, %d, %d", prediction_1, prediction_2, prediction_3);
            std::string box_text = cv::format("Person %d.", i);
            // Calculate the position for annotated text (make sure we don't
            // put illegal values in there):
            int pos_x = std::max(face_i.tl().x - 10, 0);
            int pos_y = std::max(face_i.tl().y - 10, 0);
            // And now put it into the image:
            cv::putText(original, box_text, cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);

            // Show the result:
            cv::imshow("face_recognizer", original);

            char key = (char) cv::waitKey(20);


            std::string human;
            std::cout << "Please enter your name, person " << i << ':';
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
            std::string output = pathToRobotGreeter + "/data/images/" + human + '_' + str + ".jpg";
            imwrite(output, face_resized);

            if (index == -1)
            {
                std::string str2;
                humans++;
                std::ostringstream temp2;
                temp2 << humans - 1;
                str2 = temp2.str();

                std::ofstream fout ((pathToRobotGreeter + "/data/names.txt").c_str(), std::fstream::app);
                fout << human << std::endl;
                fout.close();

                std::ofstream fout2 ((pathToRobotGreeter + "/data/TrainingData.csv").c_str(), std::fstream::app);
                fout2 << output + ';' + str2 << std::endl;
                fout2.close();
            }
            else
            {
                std::string str2;
                std::ostringstream temp2;
                temp2 << index;
                str2 = temp2.str();
                std::ofstream fout2 ((pathToRobotGreeter + "/data/TrainingData.csv").c_str(), std::fstream::app);
                fout2 << output + ';' + str2 << std::endl;
                fout2.close();
            }

            std::ofstream fout3 ((pathToRobotGreeter + "/data/humans.txt").c_str());
            fout3 << humans << ' ' << faces2 << std::endl;

            msg2.data = "focus";
            chatter_pub.publish(msg2);
            ros::spinOnce();
        }

        // And display it:
        char key = (char) cv::waitKey(20);
        // Exit this loop on escape:
        if (key == 27)
            return 0;
    }
    while (ros::ok())
    {
        std::cout << "Sending scan signal." << std::endl;
        std_msgs::String msg;
        msg.data = "scan";
        chatter_pub.publish(msg);
        //ros::spinOnce();

        reco = false;
        std::vector<std::string> humanNames;

        std::ifstream fin2 ((pathToRobotGreeter + "/data/names.txt").c_str());
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
            std::cout << "Face detected." << std::endl;
            reco = true;
            cv::Rect face_i = faces[i];

            cv::Point center = cv::Point((face_i.tl().x + face_i.br().x) * 0.5, (face_i.tl().y + face_i.br().y) * 0.5);

            std_msgs::String msg2;
            msg2.data = "focus_h";
            chatter_pub.publish(msg2);
            ros::spinOnce();

            std_msgs::Bool msg_b;
            int dif = center.x - 360;
            std::cout << "Focusing Horizontally." << std::endl;
            while (std::abs(dif) > 5 && ros::ok()) // false is move left
            {
                msg_b.data = dif > 0?false:true;
                chatter2_pub.publish(msg_b);
                std::string dir;
                if (msg_b.data)
                    dir = "Right";
                else
                    dir = "Left";
                std::cout << "Direction: " << dir << std::endl;
                ros::spinOnce();
                ros::Subscriber sub = n3.subscribe("fail", 1000, returnMessage);
                if (ScanMsg != 'g')
                    break;

                cap >> frame;
                cv::Mat original = frame.clone();
                // Convert the current frame to grayscale:
                cv::Mat gray;
                cvtColor(original, gray, CV_BGR2GRAY);
                // Find the faces in the frame:
                haar_cascade.detectMultiScale(gray, faces, 1.1, 10);
                face_i = faces[i];
                center = cv::Point((face_i.tl().x + face_i.br().x) * 0.5, (face_i.tl().y + face_i.br().y) * 0.5);
                dif = center.x - 360;
            }

            if (ScanMsg != 'g')
                break;

            msg2.data = "focus_v";
            chatter2_pub.publish(msg_b);
            ros::spinOnce();

            dif = center.y - 360;            
            std::cout << "Focusing Vertically." << std::endl;
            while (std::abs(dif) > 5 && ros::ok()) // false is move up
            {
                msg_b.data = dif > 0?false:true;
                chatter2_pub.publish(msg_b);                                
                std::string dir;
                if (msg_b.data)
                    dir = "Down";
                else
                    dir = "Up";
                std::cout << "Direction: " << dir << std::endl;

                if (ScanMsg != 'g')
                    break;

                ros::spinOnce();
                ros::Subscriber sub = n3.subscribe("fail", 1000, returnMessage);


                cap >> frame;
                cv::Mat original = frame.clone();
                // Convert the current frame to grayscale:
                cv::Mat gray;
                cvtColor(original, gray, CV_BGR2GRAY);
                // Find the faces in the frame:
                haar_cascade.detectMultiScale(gray, faces, 1.1, 10);
                face_i = faces[i];
                center = cv::Point((face_i.tl().x + face_i.br().x) * 0.5, (face_i.tl().y + face_i.br().y) * 0.5);
                dif = center.y - 360;
            }

            if (ScanMsg != 'g')
                break;


            // Process face by face:
            face_i = faces[i];
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
            std::string box_text;
            if (prediction_3 == -1)
                box_text = cv::format("Person %d.", i);
            else
                box_text = humanNames[prediction_3];
            // Calculate the position for annotated text (make sure we don't
            // put illegal values in there):
            int pos_x = std::max(face_i.tl().x - 10, 0);
            int pos_y = std::max(face_i.tl().y - 10, 0);
            // And now put it into the image:
            cv::putText(original, box_text, cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);

            // Show the result:
            cv::imshow("face_recognizer", original);

            char key = (char) cv::waitKey(20);
            if (prediction_3 == -1)
            {
                std::string human;
                std::cout << "Please enter your name, person " << i << ':';
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
                std::string output = pathToRobotGreeter + "/data/images/" + human + '_' + str + ".jpg";
                imwrite(output, face_resized);

                if (index == -1)
                {
                    std::string str2;
                    humans++;
                    std::ostringstream temp2;
                    temp2 << humans - 1;
                    str2 = temp2.str();

                    std::ofstream fout ((pathToRobotGreeter + "/data/names.txt").c_str(), std::fstream::app);
                    fout << human << std::endl;
                    fout.close();

                    std::ofstream fout2 ((pathToRobotGreeter + "/data/TrainingData.csv").c_str(), std::fstream::app);
                    fout2 << output + ';' + str2 << std::endl;
                    fout2.close();
                }
                else
                {
                    std::string str2;
                    std::ostringstream temp2;
                    temp2 << index;
                    str2 = temp2.str();
                    std::ofstream fout2 ((pathToRobotGreeter + "/data/TrainingData.csv").c_str(), std::fstream::app);
                    fout2 << output + ';' + str2 << std::endl;
                    fout2.close();
                }

                std::ofstream fout3 ((pathToRobotGreeter + "/data/humans.txt").c_str());
                fout3 << humans << ' ' << faces2 << std::endl;
            }


        }

        // And display it:
        if (reco)
        {
            char key = (char) cv::waitKey(20);
            // Exit this loop on escape:
            if (key == 27)
                break;
        }
    }
    return 0;
}