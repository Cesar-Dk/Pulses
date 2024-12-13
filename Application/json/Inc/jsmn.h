/**
  ******************************************************************************
  * @file           jsmn.h
  * @author 		Datakorum Development Team
  * @brief          Header file for jsmn.c file.
  *
  * @note jsmn (pronounced like 'jasmine') is a minimalistic JSON parser in C.
  * It can be easily integrated into resource-limited or embedded projects.
  *
  * @see https://github.com/zserge/jsmn
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Datakorum Solution SL.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef __JSMN_H_
#define __JSMN_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @enum
 * @brief JSON type identifier. Basic types are:
 * 	o Object
 * 	o Array
 * 	o String
 * 	o Other primitive: number, boolean (true/false) or null
 */
typedef enum {
	JSMN_PRIMITIVE = 0,/**< JSMN_PRIMITIVE */
	JSMN_OBJECT = 1,   /**< JSMN_OBJECT */
	JSMN_ARRAY = 2,    /**< JSMN_ARRAY */
	JSMN_STRING = 3    /**< JSMN_STRING */
} jsmntype_t;

/**
 * @enum JSON ERROR type. Basic types are:
 * 	o NOMEM - Not enough tokens.
 * 	o INVAL - Invalid character.
 * 	o PART - More bytes expected.
 */
typedef enum {
	JSMN_ERROR_NOMEM = -1,	/**< JSMN_ERROR_NOMEM - Not enough tokens were provided */
	JSMN_ERROR_INVAL = -2,	/**< JSMN_ERROR_INVAL - Invalid character inside JSON string */
	JSMN_ERROR_PART = -3  	/**< JSMN_ERROR_PART - 	The string is not a full JSON packet, more bytes expected */
} jsmnerr_t;

/**
 * JSON token description.
 * @param		type	type (object, array, string etc.)
 * @param		start	start position in JSON data string
 * @param		end		end position in JSON data string
 */
typedef struct {
	jsmntype_t type;
	int start;
	int end;
	int size;
#ifdef JSMN_PARENT_LINKS
	int parent;
#endif
} jsmntok_t;

/**
 * @struct
 * @brief JSON parser. Contains an array of token blocks available. Also stores
 * the string being parsed now and current position in that string
 *
 */
typedef struct {
	unsigned int pos;	 	/**!< offset in the JSON string */
	unsigned int toknext;	/**!< next token to allocate */
	int toksuper; 			/**!< superior token node, e.g parent object or array */
} jsmn_parser;

/**
 * @fn void jsmn_init(jsmn_parser*)
 * @brief Create JSON parser over an array of tokens
 *
 * @pre
 * @post
 * @param parser
 */
void jsmn_init(jsmn_parser *parser);

/**
 * Run JSON parser. It parses a JSON data string into and array of tokens, each describing
 * a single JSON object.
 */
int jsmn_parse(jsmn_parser *parser, const char *js, unsigned int len,
		jsmntok_t *tokens, unsigned int num_tokens);

#ifdef __cplusplus
}
#endif

#endif /* __JSMN_H_ */
