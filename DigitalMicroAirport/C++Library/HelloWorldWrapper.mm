//
//  CppWrapper.m
//  DigitalMicroAirport
//
//  Created by Jussi Kalliola (TAU) on 25.10.2022.
//

#import <Foundation/Foundation.h>
#import "HelloWorldWrapper.h"
#import "HelloWorld.hpp"

@implementation HelloWorldWrapper

- (NSString *) sayHello {
    HelloWorld helloWorld;
    std::string helloWorldMessage = helloWorld.sayHello();
    return [NSString
            stringWithCString:helloWorldMessage.c_str()
            encoding:NSUTF8StringEncoding];
}

@end
