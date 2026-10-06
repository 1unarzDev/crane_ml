using System;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Sim.Utils.ReferenceEnvironments {
    /// <summary>Opt-in spectator controls. Never moves a robot or sensor camera.</summary>
    public sealed class CraneFreeCameraController : MonoBehaviour {
        public event Action<bool> ActiveChanged;
        public bool Active {get;private set;}
        Camera view;
        Vector3 velocity;
        Vector2 angularVelocity;
        float yaw,pitch;
        bool captured;
        CursorLockMode priorLock;
        bool priorVisible;

        public void Configure(Camera camera) => view=camera;
        public void SetActive(bool active) {
            if(Active==active||view==null)return;
            Active=active;velocity=Vector3.zero;angularVelocity=Vector2.zero;
            if(active) {
                view.orthographic=false;
                yaw=view.transform.eulerAngles.y;
                pitch=Mathf.DeltaAngle(0,view.transform.eulerAngles.x);
            }else ReleaseCursor();
            ActiveChanged?.Invoke(active);
        }
        void Update() {
            if(view==null)return;
            var mouse=Mouse.current;
            if(mouse!=null&&mouse.rightButton.wasPressedThisFrame)SetActive(true);
            if(!Active)return;
            bool drag=mouse!=null&&mouse.rightButton.isPressed;
            if(drag&&!captured) {
                priorLock=Cursor.lockState;priorVisible=Cursor.visible;captured=true;
                Cursor.lockState=CursorLockMode.Locked;Cursor.visible=false;
            }else if(!drag)ReleaseCursor();
            var keyboard=Keyboard.current;
            Vector3 movement=Vector3.zero;
            Vector2 look=Vector2.zero;
            bool fast=false;
            if(keyboard!=null) {
                movement=new Vector3((keyboard.dKey.isPressed?1:0)-(keyboard.aKey.isPressed?1:0),
                    (keyboard.eKey.isPressed?1:0)-(keyboard.qKey.isPressed?1:0),
                    (keyboard.wKey.isPressed?1:0)-(keyboard.sKey.isPressed?1:0));
                look=new Vector2((keyboard.rightArrowKey.isPressed?1:0)-(keyboard.leftArrowKey.isPressed?1:0),
                    (keyboard.downArrowKey.isPressed?1:0)-(keyboard.upArrowKey.isPressed?1:0))*75;
                fast=keyboard.leftShiftKey.isPressed||keyboard.rightShiftKey.isPressed;
            }
            float dt=Mathf.Min(Time.unscaledDeltaTime,.1f);
            if(drag) {
                Vector2 delta=mouse.delta.ReadValue();
                look+=new Vector2(delta.x,-delta.y)*.12f/Mathf.Max(dt,.001f);
            }
            Step(movement,look,fast,dt);
        }
        // Explicit stepping also permits deterministic camera QA without synthetic keys.
        public void Step(Vector3 movement,Vector2 lookDegreesPerSecond,bool fast,float dt) {
            if(!Active||view==null||dt<=0)return;
            float blend=1-Mathf.Exp(-12*dt);
            angularVelocity=Vector2.Lerp(angularVelocity,lookDegreesPerSecond,blend);
            yaw+=angularVelocity.x*dt;pitch=Mathf.Clamp(pitch+angularVelocity.y*dt,-85,85);
            view.transform.rotation=Quaternion.Euler(pitch,yaw,0);
            Vector3 direction=view.transform.right*movement.x+Vector3.up*movement.y+view.transform.forward*movement.z;
            Vector3 desired=Vector3.ClampMagnitude(direction,1)*(fast?7:3.5f);
            velocity=Vector3.Lerp(velocity,desired,blend);
            view.transform.position+=velocity*dt;
        }
        void ReleaseCursor() {
            if(!captured)return;
            Cursor.lockState=priorLock;Cursor.visible=priorVisible;captured=false;
        }
        void OnDisable(){ReleaseCursor();}
        void OnDestroy(){ReleaseCursor();}
    }
}
