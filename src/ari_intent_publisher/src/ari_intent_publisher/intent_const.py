class IntentConst:
        """Definitions of constant variables for /intents"""
        ENGAGE_WITH="__intent_engage_with__"
        MOVE_TO="__intent_move_to__"
        GUIDE="__intent_guide__"
        GRAB_OBJECT="__intent_grab_object__"
        BRING_OBJECT="__intent_bring_object__"
        PLACE_OBJECT="__intent_place_object__"
        GREET="__intent_greet__"
        SAY="__intent_say__"
        PRESENT_CONTENT="__intent_present_content__"
        ANSWER_CONTENT="__intent_answer_content__"
        PERFORM_MOTION="__intent_perform_motion__"
        START_ACTIVITY="__intent_start_activity__"
        STOP_ACTIVITY="__intent_stop_activity__"
        ENGAGE_USER="__intent_engage_user__"
        DISENGAGE_USER="__intent_disengage_user__"

        def ToIntentConst(string:str) -> str:
            """Convert a string literal to corresponding variable.
            If string is invalid, returns 'None'."""
            if string == 'ENGAGE_WITH':
                return IntentConst.ENGAGE_WITH
            
            elif string == 'MOVE_TO':
                return IntentConst.MOVE_TO
            
            elif string == 'GUIDE':
                return IntentConst.GUIDE
                
            elif string == 'GRAB_OBJECT':
                return IntentConst.GRAB_OBJECT
                
            elif string == 'BRING_OBJECT':
                return IntentConst.BRING_OBJECT
                
            elif string == 'PLACE_OBJECT':
                return IntentConst.PLACE_OBJECT
                
            elif string == 'GREET':
                return IntentConst.GREET
                
            elif string == 'SAY':
                return IntentConst.SAY
                
            elif string == 'PRESENT_CONTENT':
                return IntentConst.PRESENT_CONTENT
                
            elif string == 'PERFORM_MOTION':
                return IntentConst.PERFORM_MOTION
                
            elif string == 'START_ACTIVITY':
                return IntentConst.START_ACTIVITY
                
            elif string == 'STOP_ACTIVITY':
                return IntentConst.STOP_ACTIVITY
            
            elif string == 'ANSWER_CONTENT':
                return IntentConst.ANSWER_CONTENT
            
            elif string == "ENGAGE_USER":
                return IntentConst.ENGAGE_USER
            
            elif string == "DISENGAGE_USER":
                return IntentConst.DISENGAGE_USER
            
            else:
                return 'None'
                    