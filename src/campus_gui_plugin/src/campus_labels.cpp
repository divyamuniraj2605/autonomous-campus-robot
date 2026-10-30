#include <gazebo/gazebo.hh>
#include <gazebo/gui/GuiPlugin.hh>

#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/MovableText.hh>

#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QString>
#include <QTimer>

#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>


namespace gazebo
{

/*
 * Sends the selected destination to the ROS 2 side (gui_bridge node)
 * as a plain UDP datagram on localhost. No ROS libraries are needed
 * inside gzclient. Port can be changed with CAMPUS_GUI_PORT.
 */
static void sendToRobot(const std::string &message)
{
    int port = 5005;
    const char *env = std::getenv("CAMPUS_GUI_PORT");
    if (env && std::atoi(env) > 0)
    {
        port = std::atoi(env);
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        printf("[CAMPUS] Could not open UDP socket\n");
        fflush(stdout);
        return;
    }

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    sendto(sock, message.c_str(), message.size(), 0,
           reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    close(sock);
}

class CampusLabels : public GUIPlugin
{
public:

    CampusLabels()
        : GUIPlugin()
    {
        /*
         * ============================================================
         * IMPORTANT
         * ============================================================
         *
         * The GUIPlugin itself is transparent.
         * Gazebo owns and displays this overlay.
         *
         * DO NOT call:
         *
         *     this->show();
         *
         * because that creates the separate "gazebo" window
         * that we had earlier.
         */

        this->setAttribute(
            Qt::WA_TranslucentBackground
        );

        this->setAutoFillBackground(false);


        /*
         * ============================================================
         * MAIN GUI LAYOUT
         * ============================================================
         */

        QHBoxLayout *rootLayout =
            new QHBoxLayout(this);

        rootLayout->setContentsMargins(
            0,
            10,
            12,
            10
        );

        rootLayout->setSpacing(0);


        /*
         * Empty space pushes the location panel
         * to the RIGHT side.
         */

        rootLayout->addStretch(1);


        /*
         * ============================================================
         * LOCATION PANEL
         * ============================================================
         */

        QFrame *panel =
            new QFrame(this);

        panel->setObjectName(
            "CampusLocationPanel"
        );

        panel->setMinimumWidth(300);
        panel->setMaximumWidth(300);

        panel->setStyleSheet(
            "#CampusLocationPanel {"
            "background-color: rgba(20,20,20,235);"
            "border: 2px solid white;"
            "border-radius: 8px;"
            "}"
        );


        QVBoxLayout *panelLayout =
            new QVBoxLayout(panel);

        panelLayout->setContentsMargins(
            12,
            12,
            12,
            12
        );

        panelLayout->setSpacing(6);


        /*
         * ============================================================
         * TITLE
         * ============================================================
         */

        QLabel *title =
            new QLabel(
                "CAMPUS LOCATIONS",
                panel
            );

        title->setAlignment(
            Qt::AlignCenter
        );

        title->setStyleSheet(
            "QLabel {"
            "color: white;"
            "background: transparent;"
            "font-family: Arial;"
            "font-size: 17px;"
            "font-weight: bold;"
            "padding: 5px;"
            "}"
        );

        panelLayout->addWidget(title);


        /*
         * ============================================================
         * SUBTITLE
         * ============================================================
         */

        QLabel *subtitle =
            new QLabel(
                "Select destination",
                panel
            );

        subtitle->setAlignment(
            Qt::AlignCenter
        );

        subtitle->setStyleSheet(
            "QLabel {"
            "color: white;"
            "background: transparent;"
            "font-family: Arial;"
            "font-size: 12px;"
            "padding-bottom: 4px;"
            "}"
        );

        panelLayout->addWidget(subtitle);


        /*
         * ============================================================
         * SEPARATOR
         * ============================================================
         */

        QFrame *line =
            new QFrame(panel);

        line->setFrameShape(
            QFrame::HLine
        );

        line->setStyleSheet(
            "QFrame {"
            "color: white;"
            "background-color: white;"
            "min-height: 1px;"
            "max-height: 1px;"
            "}"
        );

        panelLayout->addWidget(line);


        /*
         * ============================================================
         * LOCATION BUTTONS
         * ============================================================
         *
         * These are the SAME locations already used in the project.
         *
         * Clicking a button prints:
         *
         * [CAMPUS] Destination selected: MAIN GATE
         *
         * etc.
         */

        addLocation(
            panelLayout,
            "MAIN GATE"
        );

        addLocation(
            panelLayout,
            "ADMINISTRATION BLOCK"
        );

        addLocation(
            panelLayout,
            "STUDENT ACTIVITY CENTER"
        );

        addLocation(
            panelLayout,
            "FOOD COURT"
        );

        addLocation(
            panelLayout,
            "DEPARTMENT STORE"
        );

        addLocation(
            panelLayout,
            "CENTRAL LIBRARY"
        );

        addLocation(
            panelLayout,
            "LEARNING CENTER"
        );

        addLocation(
            panelLayout,
            "SCHOOL OF BUSINESS"
        );

        addLocation(
            panelLayout,
            "SCHOOL OF ENGINEERING"
        );

        addLocation(
            panelLayout,
            "FACULTY QUARTERS"
        );

        addLocation(
            panelLayout,
            "PARKING AREA"
        );

        addLocation(
            panelLayout,
            "CRICKET GROUND"
        );

        addLocation(
            panelLayout,
            "SPORTS GROUND"
        );

        /*
         * STOP button: cancels the current navigation goal.
         */
        QPushButton *stopButton = new QPushButton("STOP ROBOT");
        stopButton->setMinimumHeight(34);
        stopButton->setMaximumHeight(34);
        stopButton->setStyleSheet(
            "QPushButton {"
            "color: white;"
            "background-color: rgb(170,40,40);"
            "border: 1px solid rgb(230,180,180);"
            "border-radius: 5px;"
            "font-family: Arial;"
            "font-size: 12px;"
            "font-weight: bold;"
            "}"
            "QPushButton:hover {"
            "background-color: rgb(210,60,60);"
            "border: 2px solid white;"
            "}"
        );
        panelLayout->addWidget(stopButton);
        connect(
            stopButton,
            &QPushButton::clicked,
            []()
            {
                printf("\n[CAMPUS] STOP requested\n");
                fflush(stdout);
                sendToRobot("STOP");
            }
        );


        /*
         * Put panel at the top-right.
         */

        rootLayout->addWidget(
            panel,
            0,
            Qt::AlignTop
        );


        /*
         * ============================================================
         * 3D LABEL TIMER
         * ============================================================
         *
         * Gazebo needs a little time to finish creating the
         * rendering scene and building visuals.
         *
         * We therefore check every 500 ms until the labels exist.
         */

        this->labelTimer =
            new QTimer(this);

        connect(
            this->labelTimer,
            &QTimer::timeout,
            this,
            &CampusLabels::create3DLabels
        );

        this->labelTimer->start(500);
    }


    ~CampusLabels()
    {
        if (this->labelTimer)
        {
            this->labelTimer->stop();
        }
    }


private:


    /*
     * ================================================================
     * LOCATION BUTTON
     * ================================================================
     */

    void addLocation(
        QVBoxLayout *layout,
        const std::string &name
    )
    {
        QPushButton *button =
            new QPushButton(
                QString::fromStdString(name)
            );

        button->setMinimumHeight(34);
        button->setMaximumHeight(34);

        button->setStyleSheet(
            "QPushButton {"
            "color: white;"
            "background-color: rgb(55,55,55);"
            "border: 1px solid rgb(180,180,180);"
            "border-radius: 5px;"
            "font-family: Arial;"
            "font-size: 12px;"
            "font-weight: bold;"
            "text-align: left;"
            "padding-left: 10px;"
            "}"

            "QPushButton:hover {"
            "color: white;"
            "background-color: rgb(85,85,85);"
            "border: 2px solid white;"
            "}"

            "QPushButton:pressed {"
            "color: white;"
            "background-color: rgb(110,110,110);"
            "}"
        );


        layout->addWidget(button);


        /*
         * Click handler.
         *
         * This is intentionally kept exactly as the
         * working click functionality.
         */

        connect(
            button,
            &QPushButton::clicked,
            [name]()
            {
                printf(
                    "\n[CAMPUS] Destination selected: %s\n",
                    name.c_str()
                );

                fflush(stdout);

                sendToRobot(name);
            }
        );
    }


    /*
     * ================================================================
     * 3D LABEL INFORMATION
     * ================================================================
     */

    struct LabelInfo
    {
        std::string modelName;
        std::string labelName;
        std::string displayName;

        /*
         * Height above the model origin.
         */
        float zHeight;

        /*
         * Text size.
         */
        float textHeight;
    };


    /*
     * ================================================================
     * ALL CAMPUS 3D LABELS
     * ================================================================
     *
     * These names correspond to the models in the SDF.
     */

    std::vector<LabelInfo> labelInfos =
    {
        {
            "main_gate",
            "campus_label_main_gate",
            "MAIN GATE",
            2.5f,
            1.0f
        },

        {
            "administration_block",
            "campus_label_administration_block",
            "ADMINISTRATION BLOCK",
            6.5f,
            1.0f
        },

        {
            "student_activity_center",
            "campus_label_student_activity_center",
            "STUDENT ACTIVITY CENTER",
            6.5f,
            1.0f
        },

        {
            "food_court",
            "campus_label_food_court",
            "FOOD COURT",
            5.5f,
            1.0f
        },

        {
            "department_store",
            "campus_label_department_store",
            "DEPARTMENT STORE",
            4.5f,
            0.9f
        },

        {
            "central_library",
            "campus_label_central_library",
            "CENTRAL LIBRARY",
            7.0f,
            1.0f
        },

        {
            "learning_center",
            "campus_label_learning_center",
            "LEARNING CENTER",
            6.5f,
            1.0f
        },

        {
            "school_business",
            "campus_label_school_business",
            "SCHOOL OF BUSINESS",
            6.5f,
            1.0f
        },

        {
            "school_engineering",
            "campus_label_school_engineering",
            "SCHOOL OF ENGINEERING",
            6.5f,
            1.0f
        },

        {
            "faculty_quarters",
            "campus_label_faculty_quarters",
            "FACULTY QUARTERS",
            6.5f,
            1.0f
        },

        {
            "parking_area",
            "campus_label_parking_area",
            "PARKING AREA",
            2.5f,
            0.9f
        },

        {
            "cricket_ground",
            "campus_label_cricket_ground",
            "CRICKET GROUND",
            1.5f,
            0.9f
        },

        {
            "sports_ground",
            "campus_label_sports_ground",
            "SPORTS GROUND",
            1.5f,
            0.9f
        }
    };


    /*
     * ================================================================
     * FIND VISUAL
     * ================================================================
     *
     * Gazebo visual names can sometimes be:
     *
     * model
     *
     * model::link
     *
     * model::link::visual
     *
     * Therefore we try several useful possibilities.
     */

    gazebo::rendering::VisualPtr
    findVisual(
        gazebo::rendering::ScenePtr scene,
        const std::string &modelName
    )
    {
        /*
         * First try the model name directly.
         */

        gazebo::rendering::VisualPtr visual =
            scene->GetVisual(modelName);

        if (visual)
        {
            return visual;
        }


        /*
         * Try common link name.
         */

        visual =
            scene->GetVisual(
                modelName + "::building"
            );

        if (visual)
        {
            return visual;
        }


        /*
         * Try common "link" name.
         */

        visual =
            scene->GetVisual(
                modelName + "::link"
            );

        if (visual)
        {
            return visual;
        }


        return gazebo::rendering::VisualPtr();
    }


    /*
     * ================================================================
     * CREATE 3D LABELS
     * ================================================================
     */

    void create3DLabels()
    {
        /*
         * Get Gazebo rendering scene.
         */

        gazebo::rendering::ScenePtr scene =
            gazebo::rendering::get_scene();


        if (!scene)
        {
            /*
             * Rendering scene isn't ready yet.
             * The timer will call us again.
             */

            return;
        }


        int numberCreated = 0;


        /*
         * Process every campus location.
         */

        for (
            const LabelInfo &info :
            this->labelInfos
        )
        {
            /*
             * Find the building/area visual.
             */

            gazebo::rendering::VisualPtr visual =
                findVisual(
                    scene,
                    info.modelName
                );


            if (!visual)
            {
                /*
                 * Visual hasn't appeared in Gazebo yet.
                 *
                 * Don't print this every 500 ms because that
                 * would flood the terminal.
                 */

                continue;
            }


            /*
             * Don't create the same label twice.
             */

            if (
                visual->HasAttachedObject(
                    info.labelName
                )
            )
            {
                numberCreated++;
                continue;
            }


            /*
             * Get the Ogre scene node belonging to
             * the building.
             */

            Ogre::SceneNode *buildingNode =
                visual->GetSceneNode();


            if (!buildingNode)
            {
                continue;
            }


            /*
             * ========================================================
             * CREATE A CHILD NODE ABOVE THE BUILDING
             * ========================================================
             */

            Ogre::SceneNode *labelNode =
                buildingNode->createChildSceneNode();


            labelNode->setPosition(
                Ogre::Vector3(
                    0.0f,
                    0.0f,
                    info.zHeight
                )
            );


            /*
             * ========================================================
             * CREATE GAZEBO 3D MOVABLE TEXT
             * ========================================================
             */

            gazebo::rendering::MovableText *text =
                new gazebo::rendering::MovableText();


            /*
             * White text.
             */

            ignition::math::Color white(
                1.0,
                1.0,
                1.0,
                1.0
            );


            text->Load(
                info.labelName,
                info.displayName,
                "Arial",
                info.textHeight,
                white
            );


            /*
             * Make absolutely sure the text is white.
             */

            text->SetColor(
                white
            );


            /*
             * Set text size.
             */

            text->SetCharHeight(
                info.textHeight
            );


            /*
             * Keep text visible even when it would normally
             * be hidden behind another object.
             */

            text->SetShowOnTop(
                true
            );


            /*
             * Center the label horizontally.
             */

            text->SetTextAlignment(
                gazebo::rendering::MovableText::H_CENTER,
                gazebo::rendering::MovableText::V_ABOVE
            );


            /*
             * Attach text to the building's label node.
             *
             * MovableText automatically behaves as a camera-facing
             * 3D text object.
             */

            labelNode->attachObject(
                text
            );


            /*
             * ========================================================
             * TERMINAL CONFIRMATION
             * ========================================================
             */

            printf(
                "[CAMPUS] 3D label created: %s\n",
                info.displayName.c_str()
            );

            fflush(stdout);


            numberCreated++;
        }


        /*
         * ============================================================
         * ALL LABELS CREATED
         * ============================================================
         */

        if (
            numberCreated ==
            static_cast<int>(
                this->labelInfos.size()
            )
        )
        {
            printf(
                "[CAMPUS] All 3D campus labels created.\n"
            );

            fflush(stdout);


            /*
             * No need to keep checking.
             */

            if (this->labelTimer)
            {
                this->labelTimer->stop();
            }
        }
    }


private:

    /*
     * Timer used to wait for the Gazebo rendering scene.
     */

    QTimer *labelTimer = nullptr;
};


/*
 * ================================================================
 * REGISTER GUI PLUGIN
 * ================================================================
 */

GZ_REGISTER_GUI_PLUGIN(
    CampusLabels
)

}
