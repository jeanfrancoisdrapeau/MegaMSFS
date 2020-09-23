using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using System.Diagnostics;
using Microsoft.FlightSimulator.SimConnect;
using System.Runtime.InteropServices;

namespace MegaMSFS
{
    public partial class Form1 : Form
    {
        public ArduinoControllerMain Acm;
        public SerialPort activeSerial;

        private long nbFromSimconnect = 0;

        public Form1()
        {
            InitializeComponent();
            Acm = new ArduinoControllerMain();
        }

        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            CloseConnection();
        }

        private void CloseConnection()
        {

        }

        public void DisplayInfo(string msg)
        {
            listView1.Items.Add(msg);
            listView1.Items[listView1.Items.Count - 1].EnsureVisible();
        }

        protected override void DefWndProc(ref Message m)
        {
            if (m.Msg == MySimConnect.WM_USER_SIMCONNECT)
            {
                if (MySimConnect.simconnect != null)
                {
                    try
                    {
                        MySimConnect.simconnect.ReceiveMessage();
                    }
                    catch
                    {
                    }

                }
            }
            else
            {
                base.DefWndProc(ref m);
            }
        }

        private void initDataRequest()
        {
            try
            {
                if (MySimConnect.simconnect != null)
                {

                    // listen to connect and quit msgs
                    MySimConnect.simconnect.OnRecvOpen += new SimConnect.RecvOpenEventHandler(simconnect_OnRecvOpen);
                    MySimConnect.simconnect.OnRecvQuit += new SimConnect.RecvQuitEventHandler(simconnect_OnRecvQuit);

                    // listen to exceptions
                    MySimConnect.simconnect.OnRecvException += new SimConnect.RecvExceptionEventHandler(simconnect_OnRecvException);

                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "AUTOPILOT MASTER", "bool", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "AUTOPILOT HEADING LOCK", "bool", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "AUTOPILOT APPROACH HOLD", "bool", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "AUTOPILOT NAV1 LOCK", "bool", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "AUTOPILOT ALTITUDE LOCK", "bool", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "AUTOPILOT YAW DAMPER", "bool", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "AUTOPILOT VERTICAL HOLD", "bool", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);

                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "AUTOPILOT HEADING LOCK DIR", "degrees", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "AUTOPILOT ALTITUDE LOCK VAR", "feet", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "AUTOPILOT VERTICAL HOLD VAR", "feet/minute", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);

                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "FUEL TOTAL QUANTITY", "gallons", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "FUEL TOTAL CAPACITY", "gallons", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);

                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "Plane Latitude", "degrees", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "Plane Longitude", "degrees", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "Indicated Altitude", "feet", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "TRANSPONDER CODE:1", "number", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "KOHLSMAN SETTING HG", "inHg", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "Plane Heading Degrees Magnetic", "radians", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);

                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "AMBIENT WIND VELOCITY", "knots", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);
                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "AMBIENT WIND DIRECTION", "degrees", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);

                    MySimConnect.simconnect.AddToDataDefinition(MySimConnect.DEFINITIONS.Struct1, "FLAPS HANDLE PERCENT", "Percent over 100", SIMCONNECT_DATATYPE.FLOAT64, 0.0f, SimConnect.SIMCONNECT_UNUSED);

                    // IMPORTANT: register it with the simconnect managed wrapper marshaller
                    // if you skip this step, you will only receive a uint in the .dwData field.
                    MySimConnect.simconnect.RegisterDataDefineStruct<MySimConnect.Struct1>(MySimConnect.DEFINITIONS.Struct1);

                    // Events
                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.AUTOPILOT_ONOFF, "AP_MASTER");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.AUTOPILOT_ONOFF, false);
                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.ALTITUDE_ONOFF, "AP_PANEL_ALTITUDE_HOLD");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.ALTITUDE_ONOFF, false);
                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.APPROACH_ONOFF, "AP_APR_HOLD");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.APPROACH_ONOFF, false);
                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.HEADING_ONOFF, "AP_HDG_HOLD");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.HEADING_ONOFF, false);
                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.NAV_ONOFF, "AP_NAV1_HOLD");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.NAV_ONOFF, false);
                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.VS_ONOFF, "AP_PANEL_VS_ON");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.VS_ONOFF, false);
                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.YAWDAMP_ONOFF, "YAW_DAMPER_TOGGLE");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.YAWDAMP_ONOFF, false);

                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.ALTIMETER_UP, "KOHLSMAN_INC");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.ALTIMETER_UP, false);
                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.ALTIMETER_DOWN, "KOHLSMAN_DEC");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.ALTIMETER_DOWN, false);

                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.ALTITUDE_UP, "AP_ALT_VAR_INC");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.ALTITUDE_UP, false);
                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.ALTITUDE_DOWN, "AP_ALT_VAR_DEC");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.ALTITUDE_DOWN, false);

                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.VERTICAL_UP, "AP_VS_VAR_INC");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.VERTICAL_UP, false);
                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.VERTICAL_DOWN, "AP_VS_VAR_DEC");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.VERTICAL_DOWN, false);

                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.HEADING_LEFT, "HEADING_BUG_DEC");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.HEADING_LEFT, false);
                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.HEADING_RIGHT, "HEADING_BUG_INC");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.HEADING_RIGHT, false);

                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.SYNC_HEADING, "HEADING_BUG_SET");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.SYNC_HEADING, false);
                    MySimConnect.simconnect.MapClientEventToSimEvent(MySimConnect.EVENT_CTRL.SYNC_ALTITUDE, "AP_ALT_VAR_SET_ENGLISH");
                    MySimConnect.simconnect.AddClientEventToNotificationGroup(MySimConnect.GROUP_IDS.GROUP_1, MySimConnect.EVENT_CTRL.SYNC_ALTITUDE, false);

                    MySimConnect.simconnect.SetNotificationGroupPriority(MySimConnect.GROUP_IDS.GROUP_1, SimConnect.SIMCONNECT_GROUP_PRIORITY_HIGHEST);

                    // catch a simobject data request
                    MySimConnect.simconnect.OnRecvSimobjectData += new SimConnect.RecvSimobjectDataEventHandler(simconnect_OnRecvSimobjectData);

                    // Request FSX data every second
                    MySimConnect.simconnect.RequestDataOnSimObject(MySimConnect.DATA_REQUESTS.REQUEST_1
                         , MySimConnect.DEFINITIONS.Struct1
                         , SimConnect.SIMCONNECT_OBJECT_ID_USER
                         , SIMCONNECT_PERIOD.VISUAL_FRAME
                         , SIMCONNECT_DATA_REQUEST_FLAG.DEFAULT
                         , 0
                         , 0
                         , 0);
                }
            }
            catch (COMException ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        void simconnect_OnRecvSimobjectData(SimConnect sender, SIMCONNECT_RECV_SIMOBJECT_DATA data)
        {
            nbFromSimconnect += 1;
            if (!Acm.AlreadySending)
            {
                if (Acm.CurrentPort != null)
                {
                    switch ((MySimConnect.DATA_REQUESTS)data.dwRequestID)
                    {
                        case MySimConnect.DATA_REQUESTS.REQUEST_1:
                            MySimConnect.Struct1 s1 = (MySimConnect.Struct1)data.dwData[0];

                            MySimConnect.curHeading = Convert.ToUInt32(s1.heading * 180 / Math.PI);
                            MySimConnect.curAltitude = Convert.ToInt32(s1.altitude);

                            string msg;
                            msg = "<DATA;";
                            msg += s1.autopilot + ";";
                            msg += s1.yawdamp_h + ";";
                            msg += s1.heading_h + ";";
                            msg += s1.approach_h + ";";
                            msg += s1.nav_h + ";";
                            msg += s1.altitude_h + ";";
                            msg += s1.vs_h + ";";

                            msg += Convert.ToInt32(s1.altitude) + ";";
                            msg += Convert.ToInt32(s1.heading * 180 / Math.PI) + ";";
                            msg += Math.Round(s1.altimeter, 2) + ";";
                            msg += Math.Round(s1.fuel_quantity / s1.fuel_capacity * 100) + ";";
                            msg += Math.Round(s1.wind_direction) + ";";
                            msg += Math.Round(s1.wind_velocity) + ";";
                            msg += Math.Round(s1.flaps * 100) + ";";

                            msg += Convert.ToInt32(s1.heading_var) + ";";
                            msg += Convert.ToInt32(s1.altitude_var) + ";";
                            msg += Convert.ToInt32(s1.vs_var) + ";";

                            msg += (s1.autopilot == 1 ? "AP" : "--");
                            msg += (s1.yawdamp_h == 1 ? "YD" : "--");
                            msg += (s1.heading_h == 1 ? "HDG" : "---");
                            msg += (s1.approach_h == 1 ? "APR" : "---");
                            msg += (s1.nav_h == 1 ? "NAV" : "---");
                            msg += (s1.altitude_h == 1 ? "ALT" : (s1.vs_h == 1 ? "VS-" : "---"));

                            msg += ";END;>";

                            //DisplayInfo("(" + Acm.CurrentPort.PortName + "): SEND: " + msg);

                            Acm.SendValue(msg);

                            break;

                        default:
                            break;
                    }
                }
            }
        }

        void simconnect_OnRecvOpen(SimConnect sender, SIMCONNECT_RECV_OPEN data)
        {
            DisplayInfo("Established connection to SimConnect.");
        }

        // The case where the user closes FSX
        void simconnect_OnRecvQuit(SimConnect sender, SIMCONNECT_RECV data)
        {
            DisplayInfo("Lost connection to SimConnect");
        }

        void simconnect_OnRecvException(SimConnect sender, SIMCONNECT_RECV_EXCEPTION data)
        {
            DisplayInfo("Exception from SimConnect received: " + data.dwException);
        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (Acm.CurrentPort != null)
            {
                if (Acm.CurrentPort.IsOpen)
                {
                    DisplayInfo("Closing connection to Arduino");
                    Acm.PortFound = false;
                    Acm.CurrentPort.Close();
                    Acm.CurrentPort = null;

                    button1.Text = "Connect to Arduino";
                }
            }
            else
            {
                DisplayInfo("Trying to connect to Arduino on " + textBox3.Text + "...");
                Acm.SetComPort(textBox3.Text);

                DisplayInfo("Sending handshake...");
                Acm.SendHandShake();

                if (Acm.CurrentPort.IsOpen)
                    button1.Text = "Disconnect from Arduino";
            }
        }

        private void button2_Click(object sender, EventArgs e)
        {
            try
            {
                MySimConnect.simconnect = new SimConnect("MegaMSFS", this.Handle, MySimConnect.WM_USER_SIMCONNECT, null, 0);
                initDataRequest();

                button2.Enabled = false;
            }
            catch (Exception ex)
            {
                DisplayInfo("Error trying to connect to SimConnect: " + ex.Message);
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            label2.Text = nbFromSimconnect.ToString();
            label3.Text = Acm.nbFromArduino.ToString();
            label5.Text = Acm.nbToArduino.ToString();
        }
    }

    public static class MySimConnect
    {
        public const int WM_USER_SIMCONNECT = 0x0402;
        public static SimConnect simconnect;

        public static uint curHeading;
        public static int curAltitude;

        // this is how you declare a data structure so that
        // simconnect knows how to fill it/read it.
        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi, Pack = 1)]
        public struct Struct1
        {
            public double autopilot;
            public double heading_h;
            public double approach_h;
            public double nav_h;
            public double altitude_h;
            public double yawdamp_h;
            public double vs_h;

            public double heading_var;
            public double altitude_var;
            public double vs_var;

            public double fuel_quantity;
            public double fuel_capacity;

            public double latitude;
            public double longitude;
            public double altitude;
            public double transponder;
            public double altimeter;
            public double heading;

            public double wind_velocity;
            public double wind_direction;

            public double flaps;
        };

        public enum DEFINITIONS
        {
            Struct1,
        }

        public enum DATA_REQUESTS
        {
            REQUEST_1,
        };

        public enum EVENT_CTRL
        {
            AUTOPILOT_ONOFF,
            YAWDAMP_ONOFF,
            HEADING_ONOFF,
            APPROACH_ONOFF,
            NAV_ONOFF,
            ALTITUDE_ONOFF,
            VS_ONOFF,
            ALTIMETER_UP,
            ALTIMETER_DOWN,
            HEADING_LEFT,
            HEADING_RIGHT,
            ALTITUDE_DOWN,
            ALTITUDE_UP,
            VERTICAL_DOWN,
            VERTICAL_UP,
            SYNC_HEADING,
            SYNC_ALTITUDE,
        }

        public enum GROUP_IDS
        {
            GROUP_1,
        }
    }

    public class ArduinoControllerMain
    {
        public SerialPort CurrentPort;
        public bool PortFound = false;
        public bool AlreadySending = false;

        public long nbFromArduino = 0;
        public long nbToArduino = 0;

        public void SetComPort(string port)
        {
            try
            {
                if (CurrentPort != null)
                {
                    if (CurrentPort.IsOpen)
                    {
                        CurrentPort.Close();
                        CurrentPort = null;
                    }
                }

                CurrentPort = new SerialPort(port, 9600);
                CurrentPort.DataReceived += CurrentPort_DataReceived;
                CurrentPort.Open();
            }
            catch (Exception e)
            {
                PortFound = false;
                CurrentPort.Close();
            }
        }

        private void CurrentPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            nbFromArduino += 1;

            try
            {
                string inData = CurrentPort.ReadLine();
                Debug.WriteLine(inData.Replace('\r', ' '));

                switch (inData)
                {
                    case "handshakearduino\r":
                        PortFound = true;
                        AlreadySending = false;
                        break;

                    case "READY\r":
                        AlreadySending = false;
                        break;

                    case "set_autopilot_onoff\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.AUTOPILOT_ONOFF, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "set_yawdamp_onoff\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.YAWDAMP_ONOFF, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "set_heading_onoff\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.HEADING_ONOFF, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "set_approach_onoff\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.APPROACH_ONOFF, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "set_nav_onoff\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.NAV_ONOFF, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "set_altitude_onoff\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.ALTITUDE_ONOFF, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "set_vs_onoff\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.VS_ONOFF, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "set_altimeter_up\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.ALTIMETER_UP, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "set_altimeter_down\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.ALTIMETER_DOWN, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "set_heading_left\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.HEADING_LEFT, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "set_heading_right\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.HEADING_RIGHT, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "set_altitude_down\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.ALTITUDE_DOWN, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "set_altitude_up\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.ALTITUDE_UP, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "set_vertical_down\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.VERTICAL_DOWN, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "set_vertical_up\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.VERTICAL_UP, 0, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "sync_heading\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.SYNC_HEADING, MySimConnect.curHeading, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    case "sync_altitude\r":
                        if (MySimConnect.simconnect != null)
                            MySimConnect.simconnect.TransmitClientEvent(0, MySimConnect.EVENT_CTRL.SYNC_ALTITUDE, (uint)MySimConnect.curAltitude, MySimConnect.GROUP_IDS.GROUP_1, SIMCONNECT_EVENT_FLAG.GROUPID_IS_PRIORITY);
                        break;

                    default:
                        //AlreadySending = false;
                        break;
                }
            }
            catch (Exception ex)
            {
                var x = ex.Message;
            }
        }

        public void SendHandShake()
        {
            if (CurrentPort.IsOpen)
            {
                nbToArduino += 1;

                AlreadySending = true;
                CurrentPort.WriteLine("<handshakepc;END;>");
            }
        }

        public void SendValue(string msg)
        {
            if (CurrentPort.IsOpen && !AlreadySending)
            {
                try
                {
                    nbToArduino += 1;

                    AlreadySending = true;
                    CurrentPort.WriteLine(msg);
                }
                catch (Exception ex)
                {
                    var x = ex.Message;
                }
            }
        }
    }
}
