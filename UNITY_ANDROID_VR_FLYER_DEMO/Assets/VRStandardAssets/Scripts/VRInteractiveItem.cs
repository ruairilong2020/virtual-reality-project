using System;
using UnityEngine;
using UnityEngine.EventSystems;

namespace VRStandardAssets.Utils
{
    // This class should be added to any gameobject in the scene
    // that should react to input based on the user's gaze.
    // It contains events that can be subscribed to by classes that
    // need to know about input specifics to this gameobject.
    public class VRInteractiveItem : MonoBehaviour
    {


        public event Action OnOver;             // Called when the gaze moves over this object
        public event Action OnOut;              // Called when the gaze leaves this object
        public event Action OnClick;            // Called when click input is detected whilst the gaze is over this object.
        public event Action OnDoubleClick;      // Called when double click input is detected whilst the gaze is over this object.
        public event Action OnUp;               // Called when Fire1 is released whilst the gaze is over this object.
        public event Action OnDown;             // Called when Fire1 is pressed whilst the gaze is over this object.


        private EventTrigger myTrigger;

        protected bool m_IsOver;

        void Start() {
            // Try to Find an EventTrigger Script on this GameObject
            myTrigger = gameObject.GetComponent<EventTrigger>();

            // If a script does not exist..
            if (myTrigger == null) {
                Debug.Log(gameObject.name);
                // .. then create one.
                myTrigger = gameObject.AddComponent<EventTrigger>();

                // Register the Event for "Pointer Enter" (cursor goes Over button)
                EventTrigger.Entry entryOver = new EventTrigger.Entry();
                entryOver.eventID = EventTriggerType.PointerEnter;
                entryOver.callback.AddListener((eventData) => { OnOver(); });
                myTrigger.triggers.Add(entryOver);
                
                // Register the Event for "Pointer Exit" (cursor goes Out of button)
                EventTrigger.Entry entryOut = new EventTrigger.Entry();
                entryOut.eventID = EventTriggerType.PointerExit;
                entryOut.callback.AddListener((eventData) => { OnOut(); });
                myTrigger.triggers.Add(entryOut);

                // Register the Event for "Pointer Click" (physical button has been pressed down and back up)
                EventTrigger.Entry entryClick = new EventTrigger.Entry();
                entryClick.eventID = EventTriggerType.PointerClick;
                //entryClick.callback.AddListener((eventData) => { OnClick(); });
                myTrigger.triggers.Add(entryClick);

                // Register the Event for "Pointer Up" (physical button has been released)
                EventTrigger.Entry entryUp = new EventTrigger.Entry();
                entryUp.eventID = EventTriggerType.PointerUp;
                //entryUp.callback.AddListener((eventData) => { OnUp(); });
                myTrigger.triggers.Add(entryUp);
                
                // Register the Event for "Pointer Down" (physical button has been pushed down)
                EventTrigger.Entry entryDown= new EventTrigger.Entry();
                entryDown.eventID = EventTriggerType.PointerDown;
                //entryDown.callback.AddListener((eventData) => { OnDown(); });
                myTrigger.triggers.Add(entryDown);
            }
        }


        public bool IsOver
        {
            get { return m_IsOver; }              // Is the gaze currently over this object?
        }


        // The below functions are called by the VREyeRaycaster when the appropriate input is detected.
        // They in turn call the appropriate events should they have subscribers.
        public void Over()
        {
            m_IsOver = true;

            if (OnOver != null)
                OnOver();
        }


        public void Out()
        {
            m_IsOver = false;

            if (OnOut != null)
                OnOut();
        }


        public void Click()
        {
            if (OnClick != null)
                OnClick();
        }


        public void DoubleClick()
        {
            if (OnDoubleClick != null)
                OnDoubleClick();
        }


        public void Up()
        {
            if (OnUp != null)
                OnUp();
        }


        public void Down()
        {
            if (OnDown != null)
                OnDown();
        }
    }
}