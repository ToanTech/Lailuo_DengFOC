#include "Command.h"

void Command::run(char* str){
  for(int i=0; i < call_count; i++){
        if(isSentinel(call_ids[i],str)){  // case :   call_ids = "T2"   str = "T215.15" 
          call_list[i](str+strlen(call_ids[i]));  // get 15.15  input function 
          break;
        }
      }
}
void Command::add(char* id, CommandCallback onCommand){
  call_list[call_count] = onCommand;
  call_ids[call_count] = id;
  call_count++;
}
void Command::scalar(float* value,  char* user_cmd){
  *value = atof(user_cmd);
}
bool Command::isSentinel(char* ch,char* str)
{
  char s[strlen(ch)+1];
  strncpy(s,str,strlen(ch));
  s[strlen(ch)] = '\0'; //strncpy need add end '\0'     
  if(strcmp(ch, s) == 0)
      return true;
  else 
      return false;
}
