#include "Marlin.h"
#include "cardreader.h"
#include "ultralcd.h"
#include "stepper.h"
#include "temperature.h"
#include "language.h"

#ifdef SDSUPPORT



CardReader::CardReader()
{
   filesize = 0;
   sdpos = 0;
   sdprinting = false;
   cardOK = false;
   saving = false;
   logging = false;
   autostart_atmillis=0;
   workDirDepth = 0;
   memset(workDirParents, 0, sizeof(workDirParents));

   autostart_stilltocheck=true;
   lastnr=0;

  #if SDPOWER > -1
    SET_OUTPUT(SDPOWER);
    WRITE(SDPOWER,HIGH);
  #endif

  autostart_atmillis=millis()+5000;
}

char *createFilename(char *buffer,const dir_t &p)
{
  char *pos=buffer;
  for (uint8_t i = 0; i < 11; i++)
  {
    if (p.name[i] == ' ')continue;
    if (i == 8)
    {
      *pos++='.';
    }
    *pos++=p.name[i];
  }
  *pos++=0;
  return buffer;
}


void CardReader::lsDive(const char *prepend,SdFile parent)
{
  dir_t p;
 uint8_t cnt=0;

  while (parent.readDir(p, longFilename) > 0)
  {
    if( DIR_IS_SUBDIR(&p) && lsAction!=LS_Count && lsAction!=LS_GetFilename)
    {

      char path[13*2];
      char lfilename[13];
      createFilename(lfilename,p);

      path[0]=0;
      if(strlen(prepend)==0)
      {
       strcat(path,"/");
      }
      strcat(path,prepend);
      strcat(path,lfilename);
      strcat(path,"/");



      SdFile dir;
      if(!dir.open(parent,lfilename, O_READ))
      {
        if(lsAction==LS_SerialPrint)
        {
          SERIAL_ECHO_START;
          SERIAL_ECHOLN(MSG_SD_CANT_OPEN_SUBDIR);
          SERIAL_ECHOLN(lfilename);
        }
      }
      lsDive(path,dir);



    }
    else
    {
      if (p.name[0] == DIR_NAME_FREE) break;
      if (p.name[0] == DIR_NAME_DELETED || p.name[0] == '.'|| p.name[0] == '_') continue;
      if (longFilename[0] != '\0' &&
          (longFilename[0] == '.' || longFilename[0] == '_')) continue;
      if ( p.name[0] == '.')
      {
        if ( p.name[1] != '.')
        continue;
      }

      if (!DIR_IS_FILE_OR_SUBDIR(&p)) continue;
      filenameIsDir=DIR_IS_SUBDIR(&p);


      if(!filenameIsDir)
      {
        if(p.name[8]!='G') continue;
        if(p.name[9]=='~') continue;
      }

      createFilename(filename,p);
      if(lsAction==LS_SerialPrint)
      {
        SERIAL_PROTOCOL(prepend);
        SERIAL_PROTOCOLLN(filename);
      }
      else if(lsAction==LS_Count)
      {
        nrFiles++;
      }
      else if(lsAction==LS_GetFilename)
      {
        if(cnt==nrFiles)
          return;
        cnt++;

      }
    }
  }
}

void CardReader::ls()
{
  lsAction=LS_SerialPrint;
  if(lsAction==LS_Count)
  nrFiles=0;

  root.rewind();
  lsDive("",root);
}


void CardReader::initsd()
{
  cardOK = false;
  if(root.isOpen())
    root.close();
#ifdef SDSLOW
  if (!card.init(SPI_HALF_SPEED,SDSS))
#else
  if (!card.init(SPI_FULL_SPEED,SDSS))
#endif
  {

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM(MSG_SD_INIT_FAIL);
  }
  else if (!volume.init(&card))
  {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_SD_VOL_INIT_FAIL);
  }
  else if (!root.openRoot(&volume))
  {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_SD_OPENROOT_FAIL);
  }
  else
  {
    cardOK = true;
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM(MSG_SD_CARD_OK);
  }
  workDir=root;
  curDir=&root;







}

void CardReader::setroot()
{




  workDir=root;

  curDir=&workDir;
}
void CardReader::release()
{
  sdprinting = false;
  cardOK = false;
}

void CardReader::startFileprint()
{
  if(cardOK)
  {
    sdprinting = true;
  }
}

void CardReader::pauseSDPrint()
{
  if(sdprinting)
  {
    sdprinting = false;
  }
}


void CardReader::openLogFile(char* name)
{
  logging = true;
  openFile(name, false);
}

void CardReader::openFile(char* name,bool read)
{
  if(!cardOK)
    return;
  file.close();
  sdprinting = false;


  SdFile myDir;
  curDir=&root;
  char *fname=name;

  char *dirname_start,*dirname_end;
  if(name[0]=='/')
  {
    dirname_start=strchr(name,'/')+1;
    while(dirname_start>0)
    {
      dirname_end=strchr(dirname_start,'/');


      if(dirname_end>0 && dirname_end>dirname_start)
      {
        char subdirname[13];
        strncpy(subdirname, dirname_start, dirname_end-dirname_start);
        subdirname[dirname_end-dirname_start]=0;
        SERIAL_ECHOLN(subdirname);
        if(!myDir.open(curDir,subdirname,O_READ))
        {
          SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
          SERIAL_PROTOCOL(subdirname);
          SERIAL_PROTOCOLLNPGM(".");
          return;
        }
        else
        {

        }

        curDir=&myDir;
        dirname_start=dirname_end+1;
      }
      else
      {
        fname=dirname_start;


        break;
      }

    }
  }
  else
  {
    curDir=&workDir;
  }
  if(read)
  {
    if (file.open(curDir, fname, O_READ))
    {
      filesize = file.fileSize();
      SERIAL_PROTOCOLPGM(MSG_SD_FILE_OPENED);
      SERIAL_PROTOCOL(fname);
      SERIAL_PROTOCOLPGM(MSG_SD_SIZE);
      SERIAL_PROTOCOLLN(filesize);
      sdpos = 0;

      SERIAL_PROTOCOLLNPGM(MSG_SD_FILE_SELECTED);
      lcd_setstatus(fname);
    }
    else
    {
      SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
      SERIAL_PROTOCOL(fname);
      SERIAL_PROTOCOLLNPGM(".");
    }
  }
  else
  {
    if (!file.open(curDir, fname, O_CREAT | O_APPEND | O_WRITE | O_TRUNC))
    {
      SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
      SERIAL_PROTOCOL(fname);
      SERIAL_PROTOCOLLNPGM(".");
    }
    else
    {
      saving = true;
      SERIAL_PROTOCOLPGM(MSG_SD_WRITE_TO_FILE);
      SERIAL_PROTOCOLLN(name);
      lcd_setstatus(fname);
    }
  }

}

void CardReader::removeFile(char* name)
{
  if(!cardOK)
    return;
  file.close();
  sdprinting = false;


  SdFile myDir;
  curDir=&root;
  char *fname=name;

  char *dirname_start,*dirname_end;
  if(name[0]=='/')
  {
    dirname_start=strchr(name,'/')+1;
    while(dirname_start>0)
    {
      dirname_end=strchr(dirname_start,'/');


      if(dirname_end>0 && dirname_end>dirname_start)
      {
        char subdirname[13];
        strncpy(subdirname, dirname_start, dirname_end-dirname_start);
        subdirname[dirname_end-dirname_start]=0;
        SERIAL_ECHOLN(subdirname);
        if(!myDir.open(curDir,subdirname,O_READ))
        {
          SERIAL_PROTOCOLPGM("open failed, File: ");
          SERIAL_PROTOCOL(subdirname);
          SERIAL_PROTOCOLLNPGM(".");
          return;
        }
        else
        {

        }

        curDir=&myDir;
        dirname_start=dirname_end+1;
      }
      else
      {
        fname=dirname_start;


        break;
      }

    }
  }
  else
  {
    curDir=&workDir;
  }
    if (file.remove(curDir, fname))
    {
      SERIAL_PROTOCOLPGM("File deleted:");
      SERIAL_PROTOCOL(fname);
      sdpos = 0;
    }
    else
    {
      SERIAL_PROTOCOLPGM("Deletion failed, File: ");
      SERIAL_PROTOCOL(fname);
      SERIAL_PROTOCOLLNPGM(".");
    }

}

void CardReader::getStatus()
{
  if(cardOK){
    SERIAL_PROTOCOLPGM(MSG_SD_PRINTING_BYTE);
    SERIAL_PROTOCOL(sdpos);
    SERIAL_PROTOCOLPGM("/");
    SERIAL_PROTOCOLLN(filesize);
  }
  else{
    SERIAL_PROTOCOLLNPGM(MSG_SD_NOT_PRINTING);
  }
}
void CardReader::write_command(char *buf)
{
  char* begin = buf;
  char* npos = 0;
  char* end = buf + strlen(buf) - 1;

  file.writeError = false;
  if((npos = strchr(buf, 'N')) != NULL)
  {
    begin = strchr(npos, ' ') + 1;
    end = strchr(npos, '*') - 1;
  }
  end[1] = '\r';
  end[2] = '\n';
  end[3] = '\0';
  file.write(begin);
  if (file.writeError)
  {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_SD_ERR_WRITE_TO_FILE);
  }
}


void CardReader::checkautostart(bool force)
{
  if(!force)
  {
    if(!autostart_stilltocheck)
      return;
    if(autostart_atmillis<millis())
      return;
  }
  autostart_stilltocheck=false;
  if(!cardOK)
  {
    initsd();
    if(!cardOK)
      return;
  }

  char autoname[30];
  sprintf_P(autoname, PSTR("auto%i.g"), lastnr);
  for(int8_t i=0;i<(int8_t)strlen(autoname);i++)
    autoname[i]=tolower(autoname[i]);
  dir_t p;

  root.rewind();

  bool found=false;
  while (root.readDir(p, NULL) > 0)
  {
    for(int8_t i=0;i<(int8_t)strlen((char*)p.name);i++)
    p.name[i]=tolower(p.name[i]);



    if(p.name[9]!='~')
    if(strncmp((char*)p.name,autoname,5)==0)
    {
      char cmd[30];

      sprintf_P(cmd, PSTR("M23 %s"), autoname);
      enquecommand(cmd);
      enquecommand_P(PSTR("M24"));
      found=true;
    }
  }
  if(!found)
    lastnr=-1;
  else
    lastnr++;
}

void CardReader::closefile()
{
  file.sync();
  file.close();
  saving = false;
  logging = false;
}

void CardReader::getfilename(const uint8_t nr)
{
  curDir=&workDir;
  lsAction=LS_GetFilename;
  nrFiles=nr;
  curDir->rewind();
  lsDive("",*curDir);

}

uint16_t CardReader::getnrfilenames()
{
  curDir=&workDir;
  lsAction=LS_Count;
  nrFiles=0;
  curDir->rewind();
  lsDive("",*curDir);

  return nrFiles;
}

void CardReader::chdir(const char * relpath)
{
  SdFile newfile;
  SdFile *parent=&root;

  if(workDir.isOpen())
    parent=&workDir;

  if(!newfile.open(*parent,relpath, O_READ))
  {
   SERIAL_ECHO_START;
   SERIAL_ECHOPGM(MSG_SD_CANT_ENTER_SUBDIR);
   SERIAL_ECHOLN(relpath);
  }
  else
  {
    if (workDirDepth < MAX_DIR_DEPTH) {
      for (int d = ++workDirDepth; d--;)
        workDirParents[d+1] = workDirParents[d];
      workDirParents[0]=*parent;
    }
    workDir=newfile;
  }
}

void CardReader::updir()
{
  if(workDirDepth > 0)
  {
    --workDirDepth;
    workDir = workDirParents[0];
    int d;
    for (int d = 0; d < workDirDepth; d++)
      workDirParents[d] = workDirParents[d+1];
  }
}


void CardReader::printingHasFinished()
{
    st_synchronize();
    quickStop();
    file.close();
    sdprinting = false;
    if(SD_FINISHED_STEPPERRELEASE)
    {

        enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
    }
    autotempShutdown();
}
#endif
